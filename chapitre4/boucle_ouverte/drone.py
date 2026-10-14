#!/usr/bin/env python3
"""GEN6323 : drone dans le plan x-z, entrées prescrites sans retour d'état.

Exécution : python drone.py [--dt 0.002] [--output DOSSIER]
Figures facultatives : ajouter --figures DOSSIER_FIGURES.
Dépendances : numpy; matplotlib uniquement pour les figures.

Le tangage prescrit est theta_d=(pi/12)*sin(2*pi*t/T)**3, T=4 s.
Les demandes f_d=m*g/cos(theta_d) et tau_d=J_y*theta_ddot_d sont
calculées avant l'intégration, sans utiliser la position, l'angle ou les
vitesses réels. Le drone nominal part du repos à 5 m; il accélère puis
freine horizontalement, tout en conservant exactement son altitude.

Axes : x vers la droite, z vers le haut; le tangage positif incline la
poussée vers +x. Les groupes L et R sont à -ell et +ell sur le corps :
f=F_L+F_R et tau_y=ell*(F_L-F_R). Les états sont rangés dans l'ordre
[x,z,theta,vx,vz,theta_dot]. Le modèle est une coupe plane pédagogique,
sans traînée ni dynamique des moteurs. Il ne représente pas le vol 3D.

Trois cas : nominal non borné; groupes bornés à [0,7.7] N; masse réelle
augmentée de 5 % avec les mêmes demandes nominales non bornées. Dans
ce dernier cas, J_y est conservé pour isoler l'erreur de masse; ce n'est
pas une modélisation complète d'une charge ajoutée. L'intégration s'arrête
au sol, sans rebond. Aucun des trois cas fournis n'atteint le sol avant T.
Les sorties sont locales au programme par défaut; aucun dossier de cours
n'est nécessaire. L'import du module n'écrit aucun fichier.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import inspect
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Parameters:
    m: float = 1.5               # kg, masse nominale
    J_y: float = 0.030           # kg m², inertie de tangage
    ell: float = 0.25            # m, bras de chaque groupe équivalent
    g: float = 9.81              # m/s²
    theta_max_rad: float = np.pi/12
    z0: float = 5.0              # m, altitude initiale
    F_max: float = 7.7           # N, limite par groupe dans le cas borné
    mass_error_fraction: float = 0.05


P = Parameters()
DURATIONS = (4.0,)
MODES = ('nominal', 'limited', 'model_error')
LABELS = {'nominal': 'Nominal', 'limited': 'Poussées bornées',
          'model_error': 'Masse réelle +5 %'}
DESCRIPTIONS = {
    'nominal': 'Modèle exact, poussées prescrites non bornées.',
    'limited': 'Modèle nominal; chaque groupe est borné entre 0 et 7,7 N.',
    'model_error': 'Masse réelle +5 %; mêmes poussées nominales non bornées. Inertie de tangage inchangée.',
}
COLORS = {'nominal': '#1F5A94', 'limited': '#D95D39',
          'model_error': '#7956A5', 'reference': '#5B6975'}
STYLES = {'nominal': '-', 'limited': '--', 'model_error': '-.'}


def pitch_reference(t, T=4.0, p=P):
    """Angle, vitesse et accélération de tangage, calculés analytiquement."""
    s = 2*np.pi*np.asarray(t)/T
    k = 2*np.pi/T
    sn, cs = np.sin(s), np.cos(s)
    return np.array([p.theta_max_rad*sn**3,
                     3*p.theta_max_rad*k*sn**2*cs,
                     p.theta_max_rad*k*k*(6*sn-9*sn**3)])


def reference(t, T=4.0, p=P, quadrature_order=64):
    """État désiré; x et vx intègrent g*tan(theta_d), indépendamment du robot.

    La quadrature de Gauss évalue vx=integrale(a_x) et
    x=integrale((t-s)*a_x(s)). z_d=z0, vz_d=0; les conditions initiales
    horizontales sont nulles. Les dérivées nécessaires aux entrées sont
    analytiques dans pitch_reference, sans différences numériques.
    """
    times = np.asarray(t)
    nodes, weights = np.polynomial.legendre.leggauss(quadrature_order)
    grid = times[..., None]*(nodes+1)/2
    ax = p.g*np.tan(pitch_reference(grid, T, p)[0])
    vx = times/2*np.sum(weights*ax, axis=-1)
    x = times/2*np.sum(weights*(times[..., None]-grid)*ax, axis=-1)
    theta, q, _ = pitch_reference(times, T, p)
    return np.array([x, np.full_like(times, p.z0, dtype=float), theta,
                     vx, np.zeros_like(times, dtype=float), q])


def group_forces(f, tau, p=P):
    """Ordre L,R : groupe L à -ell, groupe R à +ell."""
    return np.array([(f+tau/p.ell)/2, (f-tau/p.ell)/2])


def generalized_efforts(forces, p=P):
    return np.array([forces[0]+forces[1], p.ell*(forces[0]-forces[1])])


def prescribed_forces(t, T=4.0, nominal=P):
    """Programme d'entrée nominal : aucun argument d'état réel."""
    theta, _, theta_ddot = pitch_reference(t, T, nominal)
    f = nominal.m*nominal.g/np.cos(theta)
    tau = nominal.J_y*theta_ddot
    return group_forces(f, tau, nominal)


def dynamics(state, efforts, actual=P):
    """Dynamique non linéaire exacte de la coupe plane, avec la gravité."""
    f, tau = efforts
    theta = state[2]
    return np.array([state[3], state[4], state[5],
                     f*np.sin(theta)/actual.m,
                     f*np.cos(theta)/actual.m-actual.g,
                     tau/actual.J_y])


def rk4_step(state, h, effort_start, effort_mid, effort_end, actual):
    k1 = dynamics(state, effort_start, actual)
    k2 = dynamics(state+h*k1/2, effort_mid, actual)
    k3 = dynamics(state+h*k2/2, effort_mid, actual)
    k4 = dynamics(state+h*k3, effort_end, actual)
    return state+h*(k1+2*k2+2*k3+k4)/6


def simulate(T=4.0, mode='nominal', dt=0.002, nominal=P, initial=None):
    if dt not in (0.001, 0.002) or not np.isfinite(T) or T <= 0:
        raise ValueError('Durée positive finie et pas 0.001 ou 0.002 requis.')
    if mode not in MODES:
        raise ValueError('Cas inconnu.')
    n = int(round(T/dt))
    if not np.isclose(n*dt, T, rtol=0, atol=1e-12):
        raise ValueError('La durée doit être un multiple du pas.')
    if not (0 < nominal.theta_max_rad < np.pi/2 and nominal.m > 0
            and nominal.J_y > 0 and nominal.ell > 0 and nominal.F_max > 0
            and nominal.mass_error_fraction > -1):
        raise ValueError('Paramètres physiques ou angle nominal invalides.')
    t = np.linspace(0, T, n+1)
    half_t = np.linspace(0, T, 2*n+1)
    # Les entrées de tous les étages RK4 sont prescrites AVANT l'intégration.
    all_requested = prescribed_forces(half_t, T, nominal).T
    all_applied = np.clip(all_requested, 0, nominal.F_max) if mode == 'limited' else all_requested.copy()
    actual = replace(nominal, m=nominal.m*(1+nominal.mass_error_fraction)) if mode == 'model_error' else nominal
    all_efforts = generalized_efforts(all_applied.T, actual).T
    state = np.empty((n+1, 6))
    first = reference(0.0, T, nominal) if initial is None else np.asarray(initial, dtype=float)
    if first.shape != (6,) or not np.isfinite(first).all() or first[1] < 0:
        raise ValueError('État initial fini à six coordonnées, au-dessus du sol, requis.')
    state[0] = first
    stop = None
    if state[0, 1] == 0:
        t, state = t[:1], state[:1]
        stop = {'t': 0.0, 'reason': 'Sol atteint : simulation arrêtée, sans rebond.'}
    else:
        for i in range(n):
            y = state[i]
            following = rk4_step(y, dt, *all_efforts[2*i:2*i+3], actual)
            if following[1] <= 0:
                # Recherche de l'événement au sein du pas; aucune loi n'est corrigée.
                def partial_step(h):
                    forces = prescribed_forces(np.array([t[i], t[i]+h/2, t[i]+h]), T, nominal)
                    if mode == 'limited':
                        forces = np.clip(forces, 0, nominal.F_max)
                    return rk4_step(y, h, *generalized_efforts(forces, actual).T, actual)
                lo, hi = 0.0, dt
                for _ in range(35):
                    mid = (lo+hi)/2
                    if partial_step(mid)[1] > 0:
                        lo = mid
                    else:
                        hi = mid
                h = (lo+hi)/2
                following = partial_step(h)
                following[1] = 0.0  # Position exacte de la surface d'événement.
                state[i+1] = following
                t[i+1] = t[i]+h
                t, state = t[:i+2], state[:i+2]
                stop = {'t': float(t[-1]), 'reason': 'Sol atteint : simulation arrêtée, sans rebond.'}
                break
            state[i+1] = following
    requested = prescribed_forces(t, T, nominal).T
    applied = np.clip(requested, 0, nominal.F_max) if mode == 'limited' else requested.copy()
    return {'T': T, 'dt': dt, 'mode': mode, 't': t, 'state': state,
            'ref': reference(t, T, nominal).T, 'requested': requested,
            'input': applied, 'requested_peak': float(np.max(all_requested)),
            'actual': asdict(actual), 'stop': stop}


def metrics(run, p=P):
    t, state, ref = run['t'], run['state'], run['ref']
    elapsed = t[-1]-t[0]
    average = lambda y: float(np.trapezoid(y, t)/elapsed) if elapsed else float(y[0])
    error = np.linalg.norm(state[:, :2]-ref[:, :2], axis=1)
    angle_error_deg = np.rad2deg(state[:, 2]-ref[:, 2])
    saturation = np.any((run['requested'] < 0) | (run['requested'] > p.F_max), axis=1)
    return {
        'position_rmse_m': average(error**2)**0.5,
        'position_max_error_m': float(error.max()),
        'position_final_error_m': float(error[-1]),
        'x_final_m': float(state[-1, 0]),
        'altitude_final_m': float(state[-1, 1]),
        'altitude_min_m': float(state[:, 1].min()),
        'pitch_max_error_deg': float(np.abs(angle_error_deg).max()),
        'pitch_final_error_deg': float(angle_error_deg[-1]),
        'vx_final_m_s': float(state[-1, 3]),
        'vz_final_m_s': float(state[-1, 4]),
        'requested_peak_force_N': run['requested_peak'],
        'applied_peak_force_N': float(np.max(run['input'])),
        'saturation_fraction': average(saturation.astype(float)) if run['mode'] == 'limited' else 0.0,
        'nominal_demand_above_limit_fraction': average(saturation.astype(float)),
        'end_time_s': float(t[-1]),
    }


def verify(runs, dt, p=P):
    """Signes, entrées prescrites, solutions nominales et raffinement du pas."""
    rng = np.random.default_rng(6323)
    efforts = rng.uniform(-30, 30, (2, 1000))
    residual = generalized_efforts(group_forces(*efforts, p), p)-efforts
    hover = dynamics(np.array([0., p.z0, 0., 0., 0., 0.]), np.array([p.m*p.g, 0.]), p)
    theta = np.deg2rad(15)
    tilted = dynamics(np.array([0., p.z0, theta, 0., 0., 0.]), np.array([p.m*p.g/np.cos(theta), 0.]), p)
    checks = {
        'allocation_1000_cases_max_error': float(np.abs(residual).max()),
        'hover_acceleration_max_error': float(np.abs(hover).max()),
        'positive_pitch_ax_m_s2': float(tilted[3]),
        'tilted_altitude_acceleration_error': float(abs(tilted[4])),
        'left_group_increase_gives_positive_moment': bool(generalized_efforts(np.array([8., 7.]), p)[1] > 0),
        'command_arguments': list(inspect.signature(prescribed_forces).parameters),
        'finite_states': all(np.isfinite(run['state']).all() for run in runs.values()),
        'reference_matches_nominal': {}, 'timestep_comparison': {},
    }
    for T in DURATIONS:
        nominal = runs[T, 'nominal']
        checks['reference_matches_nominal'][f'T{T:g}'] = np.max(np.abs(nominal['state']-nominal['ref']), axis=0).tolist()
        for mode in MODES:
            other = simulate(T, mode, 0.001 if dt == 0.002 else 0.002, p)
            coarse, fine = (runs[T, mode], other) if dt == 0.002 else (other, runs[T, mode])
            if coarse['stop'] or fine['stop']:
                raise RuntimeError('Le scénario de référence ne doit pas atteindre le sol.')
            delta = coarse['state']-fine['state'][::2]
            checks['timestep_comparison'][f'T{T:g}_{mode}'] = {
                'max_position_difference_m': float(np.linalg.norm(delta[:, :2], axis=1).max()),
                'max_pitch_difference_rad': float(np.abs(delta[:, 2]).max()),
                'max_abs_state_difference': np.max(np.abs(delta), axis=0).tolist(),
            }
    run = runs[4.0, 'model_error']
    ratio = 1/(1+p.mass_error_fraction)
    exact_x = run['ref'][:, 0]*ratio
    exact_z = p.z0+0.5*p.g*(ratio-1)*run['t']**2
    checks['mass_error_analytic_position_max_error_m'] = float(max(np.abs(run['state'][:, 0]-exact_x).max(), np.abs(run['state'][:, 1]-exact_z).max()))
    checks['reference_quadrature_64_vs_96_max_error'] = float(np.max(np.abs(reference(np.linspace(0, 4, 201), 4, p, 64)-reference(np.linspace(0, 4, 201), 4, p, 96))))
    limited = runs[4.0, 'limited']
    checks['limited_min_force_N'] = float(limited['input'].min())
    checks['limited_max_force_N'] = float(limited['input'].max())
    checks['positive_nominal_group_forces'] = bool((runs[4.0, 'nominal']['requested'] > 0).all())
    checks['identical_requested_inputs_between_cases'] = all(np.array_equal(runs[4.0, 'nominal']['requested'], runs[4.0, mode]['requested']) for mode in MODES)
    initial = reference(0., 4., p)+np.array([0.2, 1., 0.02, 0.1, 0., 0.])
    displaced = simulate(4., 'limited', dt, p, initial)
    checks['inputs_unchanged_after_initial_state_perturbation'] = bool(displaced['stop'] is None and np.array_equal(displaced['input'], limited['input']))
    checks['perturbed_trajectory_is_different'] = bool(displaced['state'].shape == limited['state'].shape and np.max(np.abs(displaced['state']-limited['state'])) > 0.1)
    falling = simulate(4., 'model_error', dt, replace(p, z0=0.1))
    expected_stop = np.sqrt(2*0.1/(p.g*(1-ratio)))
    checks['ground_stop'] = {'stop': falling['stop'], 'expected_time_s': float(expected_stop),
                             'time_error_s': float(abs(falling['t'][-1]-expected_stop)),
                             'last_altitude_m': float(falling['state'][-1, 1]),
                             'no_below_ground_state': bool((falling['state'][:, 1] >= 0).all()),
                             'times_strictly_increasing': bool((np.diff(falling['t']) > 0).all())}
    starting_on_ground = simulate(4., 'nominal', dt, replace(p, z0=0.0))
    checks['immediate_ground_stop_has_one_frame'] = bool(len(starting_on_ground['t']) == 1 and starting_on_ground['stop']['t'] == 0.0)
    checks['maximum_timestep_position_difference_m'] = max(v['max_position_difference_m'] for v in checks['timestep_comparison'].values())
    checks['passed'] = bool(
        checks['allocation_1000_cases_max_error'] < 1e-12 and checks['hover_acceleration_max_error'] < 1e-12
        and checks['positive_pitch_ax_m_s2'] > 0 and checks['tilted_altitude_acceleration_error'] < 1e-12
        and checks['left_group_increase_gives_positive_moment'] and checks['finite_states']
        and max(max(v) for v in checks['reference_matches_nominal'].values()) < 1e-7
        and checks['mass_error_analytic_position_max_error_m'] < 1e-7
        and checks['reference_quadrature_64_vs_96_max_error'] < 1e-10
        and checks['limited_min_force_N'] >= 0 and checks['limited_max_force_N'] <= p.F_max
        and checks['positive_nominal_group_forces'] and checks['identical_requested_inputs_between_cases']
        and checks['inputs_unchanged_after_initial_state_perturbation'] and checks['perturbed_trajectory_is_different']
        and checks['maximum_timestep_position_difference_m'] < 1e-4
        and checks['ground_stop']['time_error_s'] < 1e-7 and checks['ground_stop']['no_below_ground_state']
        and checks['ground_stop']['times_strictly_increasing'] and checks['immediate_ground_stop_has_one_frame'])
    if not checks['passed']:
        raise RuntimeError('Échec des vérifications : '+json.dumps(checks))
    return checks


def make_data(runs, checks, dt, p=P):
    data = {
        'id': 'quad', 'title': 'Drone dans le plan x-z : poussées prescrites',
        'units': {'time': 's', 'state': ['m', 'm', 'rad', 'm/s', 'm/s', 'rad/s'], 'input': ['N', 'N']},
        'state_names': ['x', 'z', 'theta', 'vx', 'vz', 'theta_dot'], 'input_names': ['F_L', 'F_R'],
        'durations': [4], 'parameters': asdict(p), 'integration_dt': dt, 'sample_dt': 0.02,
        'assumptions': ['Coupe plane x-z; tangage positif vers la droite, verticale montante.',
                        'Poussées programmées à partir du temps, sans retour de position, vitesse ou angle.',
                        'Départ au repos à 5 m; inclinaison désirée de ±15 degrés en 4 secondes.',
                        'Gravité physique dans tous les cas; aucune traînée ni dynamique des moteurs.',
                        'Masse réelle +5 % : inertie de tangage conservée, demandes nominales non bornées.',
                        'Après un arrêt au sol, le dernier état affiché est figé; aucun rebond ni vol stationnaire n’est simulé.'],
        'checks': checks, 'experiments': [],
    }
    arr = lambda a: np.round(a, 10).tolist()
    for T in DURATIONS:
        times = np.linspace(0, T, round(T/0.02)+1)
        experiment = {'id': f'T{T:g}', 'duration': T, 't': arr(times),
                      'reference': {'state': arr(reference(times, T, p).T),
                                    'input': arr(prescribed_forces(times, T, p).T)}, 'cases': []}
        for mode in MODES:
            run = runs[T, mode]
            # np.interp fige le dernier état si une simulation s'arrête avant T.
            states = np.column_stack([np.interp(times, run['t'], run['state'][:, k]) for k in range(6)])
            inputs = np.column_stack([np.interp(times, run['t'], run['input'][:, k]) for k in range(2)])
            experiment['cases'].append({'id': mode, 'label': LABELS[mode], 'description': DESCRIPTIONS[mode],
                                         'state': arr(states), 'input': arr(inputs), 'metrics': metrics(run, p),
                                         'actual_parameters': run['actual'], 'stop': run['stop']})
        data['experiments'].append(experiment)
    return data


def plot_results(runs, directory, p=P):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.titlesize': 14, 'axes.labelsize': 12,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'grid.alpha': 0.22, 'text.color': '#203040',
                         'axes.labelcolor': '#203040', 'legend.frameon': False,
                         'pdf.fonttype': 42, 'figure.facecolor': 'white'})
    directory.mkdir(parents=True, exist_ok=True)
    def save(fig, name):
        fig.savefig(directory/(name+'.pdf'), bbox_inches='tight', metadata={'Creator': 'GEN6323', 'CreationDate': None})
        fig.savefig(directory/(name+'.png'), dpi=180, bbox_inches='tight')
        plt.close(fig)
    def line(ax, x, y, mode):
        ax.plot(x, y, color=COLORS[mode], ls=STYLES[mode], lw=2., label=LABELS[mode])
    def legend(fig, ax):
        handles, labels = ax.get_legend_handles_labels()
        fig.legend(handles, labels, loc='outside lower center', ncol=len(labels), fontsize=10)

    base = runs[4., 'nominal']
    fig, ax = plt.subplots(figsize=(10.6, 5.1), layout='constrained')
    for mode in MODES:
        run = runs[4., mode]
        line(ax, run['state'][:, 0], run['state'][:, 1], mode)
        ax.scatter(*run['state'][-1, :2], color=COLORS[mode], s=35, zorder=4)
    ax.plot(base['ref'][:, 0], base['ref'][:, 1], ':', color=COLORS['reference'], lw=1.4, label='Référence')
    ax.scatter(0, p.z0, color='#102A43', s=32, zorder=5)
    ax.annotate('Départ au repos', xy=(0, p.z0), xytext=(0.3, p.z0+0.35), fontsize=10)
    ax.axhline(0, color='#687684', lw=1)
    ax.text(0.05, 0.1, 'Sol', color='#687684', fontsize=10)
    ax.set(xlabel='x (m)', ylabel='z (m)', title='Même programme de poussées, trois trajectoires — T = 4 s',
           xlim=(-0.35, 6.5), ylim=(-0.1, 5.65))
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    legend(fig, ax)
    save(fig, 'bo_quad_trajectoires')

    fig, axes = plt.subplots(1, 3, figsize=(12.4, 4.5), layout='constrained')
    for ax, index, scale, ylabel, title in zip(axes, (1, 2, 3), (1., 180/np.pi, 1.),
                                             ('z (m)', r'$\vartheta$ (degrés)', r'$v_x$ (m/s)'),
                                             ('Altitude', 'Tangage', 'Vitesse horizontale')):
        for mode in MODES:
            run = runs[4., mode]
            line(ax, run['t'], scale*run['state'][:, index], mode)
        ax.plot(base['t'], scale*base['ref'][:, index], ':', color=COLORS['reference'], lw=1.4, label='Référence')
        ax.set(xlabel='Temps (s)', ylabel=ylabel, title=title, xlim=(0, 4))
        ax.grid(True)
    legend(fig, axes[0])
    save(fig, 'bo_quad_etats')

    fig, axes = plt.subplots(1, 2, figsize=(11.8, 4.1), sharex=True, sharey=True, layout='constrained')
    run = runs[4., 'limited']
    for ax, k, side in zip(axes, (0, 1), ('L', 'R')):
        ax.plot(run['t'], run['requested'][:, k], color=COLORS['nominal'], lw=2,
                label='Prescrit = nominal = masse +5 %')
        ax.plot(run['t'], run['input'][:, k], color=COLORS['limited'], ls='--', lw=2,
                label='Appliqué au cas borné')
        ax.axhline(p.F_max, color=COLORS['reference'], ls=':', lw=1.3, label='Limite 7,7 N par groupe')
        ax.set(title=f'Groupe {side}', xlabel='Temps (s)', ylabel=rf'$F_{side}$ (N)', xlim=(0, 4), ylim=(7.24, 7.79))
        ax.grid(True)
    fig.suptitle(r'Détail des poussées — borne physique : $0\leq F_L,F_R\leq7{,}7$ N', fontsize=13)
    legend(fig, axes[0])
    save(fig, 'bo_quad_poussees')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dt', type=float, choices=(0.002, 0.001), default=0.002,
                        help='Pas RK4, en secondes (défaut : 0.002).')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent,
                        help='Dossier des JSON/JS et résultats; défaut : dossier du programme.')
    parser.add_argument('--figures', type=Path, help='Dossier facultatif des trois figures PDF et PNG.')
    args = parser.parse_args(argv)
    runs = {(T, mode): simulate(T, mode, args.dt) for T in DURATIONS for mode in MODES}
    checks = verify(runs, args.dt)
    data = make_data(runs, checks, args.dt)
    results = {'parameters': asdict(P), 'integration_dt': args.dt, 'sample_dt': 0.02,
               'method': 'RK4, poussées prescrites sans retour d’état; référence horizontale par quadrature de Gauss indépendante.',
               'cases': {f'T{T:g}_{mode}': metrics(runs[T, mode]) for T in DURATIONS for mode in MODES},
               'checks': checks}
    encoded = json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'quad_data.json').write_text(encoded+'\n', encoding='utf-8')
    (args.output/'quad_data.js').write_text('window.OPEN_LOOP_QUAD = '+encoded+';\n', encoding='utf-8')
    (args.output/'quad_resultats.json').write_text(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    if args.figures:
        plot_results(runs, args.figures)
    print(json.dumps({'verification_reussie': checks['passed'], 'cas': results['cases']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
