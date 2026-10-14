#!/usr/bin/env python3
"""GEN6323 : dynamique inverse d'un robot différentiel, sans retour d'état.

Exécution : python differentiel.py [--dt 0.002] [--output DOSSIER]
Figures facultatives : ajouter --figures DOSSIER_FIGURES.
Dépendances : numpy; matplotlib uniquement pour les figures.

Les couples sont prescrits à partir de la référence datée et du modèle nominal.
Aucun état mesuré n'intervient dans leur calcul. Trois cas partagent la même
pose et les mêmes vitesses initiales : nominal, couples bornés, inertie
effective de translation réelle augmentée de 10 % sans borne de couple.
Ce dernier cas isole l'erreur de modèle; il ne représente pas une charge
ajoutée complète puisque l'inertie de lacet et les résistances sont conservées.

Axes : x_B avant, y_B gauche, theta positif antihoraire; b est la voie complète.
Hypothèses : roulement sans glissement, adhérence suffisante, sol horizontal,
couples idéaux, paramètres synthétiques. Le départ a la vitesse de référence;
ce n'est pas un départ arrêté. Les sorties sont écrites dans le dossier du
programme par défaut, sans dépendance à un dossier de cours.
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
    m_e: float = 12.0       # kg, inertie effective de translation
    J_e: float = 0.70       # kg m², inertie effective de lacet
    r: float = 0.10         # m, rayon des roues
    b: float = 0.45         # m, voie complète
    c_v: float = 1.0        # N s/m, résistance longitudinale
    c_w: float = 0.08       # N m s, résistance de lacet
    tau_max: float = 1.40   # N m, limite du cas borné
    A: float = 3.0          # m, amplitude x de la référence
    B: float = 1.6          # m, amplitude y de la référence
    mass_error_fraction: float = 0.10


P = Parameters()
DURATIONS = (32.0, 8.0)
MODES = ('nominal', 'limited', 'model_error')
LABELS = {'nominal': 'Nominal', 'limited': 'Couples bornés',
          'model_error': 'Inertie réelle +10 %'}
DESCRIPTIONS = {
    'nominal': 'Modèle nominal exact; couples prescrits non bornés.',
    'limited': 'Modèle nominal; chaque couple appliqué est borné à ±1,4 N m.',
    'model_error': 'Inertie effective de translation réelle +10 %; couples nominaux non bornés. Les autres paramètres restent inchangés.',
}
COLORS = {'nominal': '#1F5A94', 'limited': '#D95D39', 'model_error': '#7956A5',
          'reference': '#5B6975'}
STYLES = {'nominal': '-', 'limited': '--', 'model_error': '-.'}


def reference(t, T, p=P):
    """x_d,y_d,theta_d,v_d,omega_d,dot(v_d),dot(omega_d); s=2*pi*t/T."""
    s = 2*np.pi*np.asarray(t)/T
    k = 2*np.pi/T
    dx, dy = p.A*np.cos(s), 2*p.B*np.cos(2*s)
    ddx, ddy = -p.A*np.sin(s), -4*p.B*np.sin(2*s)
    dddx, dddy = -p.A*np.cos(s), -8*p.B*np.cos(2*s)
    D = dx*dx + dy*dy
    N = dx*ddy - dy*ddx
    Dp = 2*(dx*ddx + dy*ddy)
    Np = dx*dddy - dy*dddx
    return np.array([p.A*np.sin(s), p.B*np.sin(2*s), np.arctan2(dy, dx),
                     k*np.sqrt(D), k*N/D,
                     k*k*(dx*ddx+dy*ddy)/np.sqrt(D),
                     k*k*(Np*D-N*Dp)/(D*D)])


def wheel_torques(F, M, p=P):
    """Couples gauche, droite à partir des efforts généralisés."""
    return np.array([p.r*F/2-p.r*M/p.b, p.r*F/2+p.r*M/p.b])


def generalized_efforts(tau, p=P):
    return np.array([(tau[0]+tau[1])/p.r,
                     p.b*(tau[1]-tau[0])/(2*p.r)])


def prescribed_torques(t, T, nominal=P):
    """Dynamique inverse nominale; aucune dépendance à l'état réel."""
    ref = reference(t, T, nominal)
    F = nominal.m_e*ref[5] + nominal.c_v*ref[3]
    M = nominal.J_e*ref[6] + nominal.c_w*ref[4]
    return wheel_torques(F, M, nominal)


def dynamics(state, efforts, actual=P):
    """Évolution réelle : les efforts appliqués sont des entrées, pas des états."""
    v, w = state[3:5]
    return np.array([v*np.cos(state[2]), v*np.sin(state[2]), w,
                     (efforts[0]-actual.c_v*v)/actual.m_e,
                     (efforts[1]-actual.c_w*w)/actual.J_e])


def simulate(T, mode, dt=0.002, nominal=P, initial=None):
    if dt not in (0.001, 0.002) or not np.isfinite(T) or T <= 0:
        raise ValueError('Durée positive finie et pas 0.001 ou 0.002 requis.')
    if mode not in MODES:
        raise ValueError('Cas inconnu.')
    n = int(round(T/dt))
    if not np.isclose(n*dt, T, rtol=0, atol=1e-12):
        raise ValueError('La durée doit être un multiple du pas.')
    t = np.linspace(0, T, n+1)
    half_t = np.linspace(0, T, 2*n+1)
    # Les entrées aux étages RK4 sont toutes calculées AVANT l'intégration.
    requested = prescribed_torques(half_t, T, nominal).T
    applied = np.clip(requested, -nominal.tau_max, nominal.tau_max) if mode == 'limited' else requested.copy()
    actual = replace(nominal, m_e=nominal.m_e*(1+nominal.mass_error_fraction)) if mode == 'model_error' else nominal
    efforts = generalized_efforts(applied.T, actual).T
    state = np.empty((n+1, 5))
    state[0] = reference(0, T, nominal)[:5] if initial is None else np.asarray(initial)
    if not np.isfinite(state[0]).all():
        raise ValueError('État initial non fini.')
    for i in range(n):
        z = state[i]
        k1 = dynamics(z, efforts[2*i], actual)
        k2 = dynamics(z+dt*k1/2, efforts[2*i+1], actual)
        k3 = dynamics(z+dt*k2/2, efforts[2*i+1], actual)
        k4 = dynamics(z+dt*k3, efforts[2*i+2], actual)
        state[i+1] = z+dt*(k1+2*k2+2*k3+k4)/6
    ref = reference(t, T, nominal).T
    # Un angle continu simplifie l'animation; seule son orientation modulo2pi compte.
    ref[:, 2] = np.unwrap(ref[:, 2])
    return {'T': T, 'dt': dt, 'mode': mode, 't': t, 'state': state, 'ref': ref,
            'requested': requested[::2], 'input': applied[::2],
            'requested_peak': float(np.max(np.abs(requested))),
            'actual': asdict(actual)}


def metrics(run, p=P):
    t, state, ref = run['t'], run['state'], run['ref']
    average = lambda y: float(np.trapezoid(y, t)/run['T'])
    error = np.linalg.norm(state[:, :2]-ref[:, :2], axis=1)
    heading_error = np.arctan2(np.sin(state[:, 2]-ref[:, 2]), np.cos(state[:, 2]-ref[:, 2]))
    saturation = np.any(np.abs(run['requested']) > p.tau_max+1e-12, axis=1)
    return {
        'position_rmse_m': average(error**2)**0.5,
        'position_max_error_m': float(error.max()),
        'position_final_error_m': float(error[-1]),
        'heading_rmse_rad': average(heading_error**2)**0.5,
        'v_rmse_m_s': average((state[:, 3]-ref[:, 3])**2)**0.5,
        'omega_rmse_rad_s': average((state[:, 4]-ref[:, 4])**2)**0.5,
        'requested_peak_torque_Nm': run['requested_peak'],
        'applied_peak_torque_Nm': float(np.max(np.abs(run['input']))),
        'saturation_fraction': average(saturation.astype(float)) if run['mode'] == 'limited' else 0.0,
        'nominal_demand_above_limit_fraction': average(saturation.astype(float)),
        'peak_reference_speed_m_s': float(np.max(ref[:, 3])),
        'initial_speed_m_s': float(ref[0, 3]),
    }


def verify(runs, dt, p=P):
    """Vérifications physiques, numériques et d'absence de retour d'état."""
    rng = np.random.default_rng(6323)
    test_efforts = rng.uniform(-40, 40, (2, 1000))
    residual = generalized_efforts(wheel_torques(*test_efforts, p), p)-test_efforts
    checks = {'allocation_max_error': float(np.max(np.abs(residual))),
              'command_arguments': list(inspect.signature(prescribed_torques).parameters),
              'reference_matches_nominal': {}, 'timestep_comparison': {}}
    checks['finite_states'] = all(np.isfinite(r['state']).all() for r in runs.values())
    checks['limited_max_torque_Nm'] = max(float(np.abs(runs[T, 'limited']['input']).max()) for T in DURATIONS)
    checks['identical_requested_inputs_between_cases'] = all(
        np.array_equal(runs[T, 'nominal']['requested'], runs[T, mode]['requested'])
        for T in DURATIONS for mode in MODES)
    for T in DURATIONS:
        run = runs[T, 'nominal']
        error = np.abs(run['state']-run['ref'][:, :5])
        checks['reference_matches_nominal'][f'T{T:g}'] = error.max(axis=0).tolist()
        for mode in MODES:
            other = simulate(T, mode, 0.001 if dt == 0.002 else 0.002, p)
            coarse, fine = (run if mode == 'nominal' else runs[T, mode], other) if dt == 0.002 else (other, runs[T, mode])
            delta = coarse['state']-fine['state'][::2]
            checks['timestep_comparison'][f'T{T:g}_{mode}'] = {
                'max_position_difference_m': float(np.linalg.norm(delta[:, :2], axis=1).max()),
                'max_abs_state_difference': np.max(np.abs(delta), axis=0).tolist(),
                'position_rmse_fine_m': metrics(fine, p)['position_rmse_m'],
            }
    initial = reference(0, 8, p)[:5] + np.array([0.2, -0.1, 0.05, 0.1, -0.03])
    displaced = simulate(8, 'limited', dt, p, initial)
    checks['inputs_unchanged_after_initial_state_perturbation'] = bool(np.array_equal(displaced['input'], runs[8, 'limited']['input']))
    checks['perturbed_trajectory_is_different'] = bool(np.max(np.abs(displaced['state']-runs[8, 'limited']['state'])) > 0.1)
    checks['slow_limited_identical_to_nominal'] = bool(np.array_equal(runs[32, 'limited']['state'], runs[32, 'nominal']['state']))
    checks['maximum_timestep_position_difference_m'] = max(x['max_position_difference_m'] for x in checks['timestep_comparison'].values())
    checks['passed'] = bool(
        checks['finite_states'] and checks['allocation_max_error'] < 1e-12
        and checks['limited_max_torque_Nm'] <= p.tau_max
        and checks['identical_requested_inputs_between_cases']
        and checks['inputs_unchanged_after_initial_state_perturbation']
        and checks['perturbed_trajectory_is_different']
        and checks['slow_limited_identical_to_nominal']
        and max(max(e) for e in checks['reference_matches_nominal'].values()) < 1e-7
        and checks['maximum_timestep_position_difference_m'] < 1e-4)
    if not checks['passed']:
        raise RuntimeError('Échec des vérifications : '+json.dumps(checks, default=float))
    return checks


def make_data(runs, checks, dt, p=P):
    data = {
        'id': 'differentiel', 'title': 'Robot différentiel : couples prescrits',
        'units': {'time': 's', 'state': ['m', 'm', 'rad', 'm/s', 'rad/s'], 'input': ['N m', 'N m']},
        'state_names': ['x', 'y', 'theta', 'v', 'omega'], 'input_names': ['tau_L', 'tau_R'],
        'durations': [32, 8], 'parameters': asdict(p), 'integration_dt': dt, 'sample_dt': 0.02,
        'assumptions': ['Couples prescrits, sans retour de pose ou de vitesse.',
                        'Pose et vitesses initiales égales à la référence; départ déjà en mouvement.',
                        'Roulement sans glissement et adhérence suffisante; paramètres pédagogiques.',
                        'Le cas inertie +10 % conserve J_e et les résistances; couples non bornés.'],
        'checks': checks, 'experiments': [],
    }
    arr = lambda a: np.round(a, 10).tolist()
    stride = round(0.02/dt)
    for T in DURATIONS:
        base = runs[T, 'nominal']
        ix = np.arange(0, len(base['t']), stride)
        experiment = {'id': f'T{T:g}', 'duration': T, 't': arr(base['t'][ix]),
                      'reference': {'state': arr(base['ref'][ix, :5]), 'input': arr(base['requested'][ix])},
                      'cases': []}
        for mode in MODES:
            run = runs[T, mode]
            experiment['cases'].append({'id': mode, 'label': LABELS[mode], 'description': DESCRIPTIONS[mode],
                                         'state': arr(run['state'][ix]), 'input': arr(run['input'][ix]),
                                         'metrics': metrics(run, p), 'actual_parameters': run['actual'], 'stop': None})
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
        ax.plot(x, y, color=COLORS[mode], ls=STYLES[mode], lw=2.0,
                label=LABELS[mode], zorder=3 if mode == 'model_error' else 2)

    def legend(fig, ax):
        handles, labels = ax.get_legend_handles_labels()
        fig.legend(handles, labels, loc='outside lower center', ncol=len(labels), fontsize=10)

    fig, axes = plt.subplots(1, 2, figsize=(12.2, 4.9), layout='constrained')
    for ax, T in zip(axes, DURATIONS):
        for mode in MODES:
            run = runs[T, mode]
            line(ax, run['state'][:, 0], run['state'][:, 1], mode)
        ref = runs[T, 'nominal']['ref']
        ax.plot(ref[:, 0], ref[:, 1], ':', lw=1.4, color=COLORS['reference'], label='Référence')
        ax.scatter([ref[0, 0]], [ref[0, 1]], color='#102A43', s=26, zorder=5)
        ax.set(title=f'T = {T:g} s', xlabel='x (m)', ylabel='y (m)')
        ax.set_aspect('equal', adjustable='box')
        ax.margins(0.08)
        ax.grid(True)
    legend(fig, axes[0])
    save(fig, 'bo_diff_trajectoires')

    fig, axes = plt.subplots(2, 2, figsize=(12.2, 6.1), sharex=True, layout='constrained')
    for j, T in enumerate(DURATIONS):
        for k in range(2):
            ax = axes[k, j]
            for mode in MODES:
                run = runs[T, mode]
                line(ax, run['t']/T, run['state'][:, 3+k], mode)
            ref = runs[T, 'nominal']
            ax.plot(ref['t']/T, ref['ref'][:, 3+k], ':', color=COLORS['reference'], lw=1.4, label='Référence')
            ax.set_ylabel('v (m/s)' if k == 0 else r'$\omega$ (rad/s)')
            ax.grid(True)
        axes[0, j].set_title(f'T = {T:g} s')
        axes[1, j].set_xlabel('Temps normalisé t / T')
    legend(fig, axes[0, 0])
    save(fig, 'bo_diff_vitesses')

    fig, axes = plt.subplots(2, 2, figsize=(12.2, 5.8), sharex=True, layout='constrained')
    for j, T in enumerate(DURATIONS):
        run = runs[T, 'limited']
        for k, side in enumerate(('L', 'R')):
            ax = axes[k, j]
            ax.plot(run['t']/T, run['requested'][:, k], color=COLORS['nominal'], lw=2,
                    label='Prescrit = nominal = inertie +10 %')
            ax.plot(run['t']/T, run['input'][:, k], color=COLORS['limited'], lw=1.9, ls='--',
                    label='Appliqué au cas borné')
            for bound in (-p.tau_max, p.tau_max):
                ax.axhline(bound, color=COLORS['reference'], lw=1, ls=':')
            ax.set_ylabel(rf'$\tau_{side}$ (N m)')
            ax.grid(True)
        axes[0, j].set_title(f'T = {T:g} s : pic prescrit {run["requested_peak"]:.2f} N m')
        axes[1, j].set_xlabel('Temps normalisé t / T')
    legend(fig, axes[0, 0])
    save(fig, 'bo_diff_couples')

    fig, axes = plt.subplots(1, 2, figsize=(12.2, 3.9), layout='constrained')
    for ax, T in zip(axes, DURATIONS):
        for mode in MODES:
            run = runs[T, mode]
            line(ax, run['t']/T, np.linalg.norm(run['state'][:, :2]-run['ref'][:, :2], axis=1), mode)
        ax.set(title=f'T = {T:g} s', xlabel='Temps normalisé t / T', ylabel='Erreur de position (m)')
        ax.grid(True)
    legend(fig, axes[0])
    save(fig, 'bo_diff_erreurs')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dt', type=float, choices=(0.002, 0.001), default=0.002,
                        help='Pas RK4, en secondes (défaut : 0.002).')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent,
                        help='Dossier des données JSON/JS et résultats; défaut : dossier du programme.')
    parser.add_argument('--figures', type=Path, help='Dossier facultatif des quatre figures PDF et PNG.')
    args = parser.parse_args(argv)
    runs = {(T, mode): simulate(T, mode, args.dt) for T in DURATIONS for mode in MODES}
    checks = verify(runs, args.dt)
    data = make_data(runs, checks, args.dt)
    results = {'parameters': asdict(P), 'integration_dt': args.dt, 'sample_dt': 0.02,
               'method': 'RK4, couples prescrits indépendants des états réels; conditions initiales exactes.',
               'cases': {f'T{T:g}_{mode}': metrics(runs[T, mode]) for T in DURATIONS for mode in MODES},
               'checks': checks}
    encoded = json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'diff_data.json').write_text(encoded+'\n', encoding='utf-8')
    (args.output/'diff_data.js').write_text('window.OPEN_LOOP_DIFF = '+encoded+';\n', encoding='utf-8')
    (args.output/'diff_resultats.json').write_text(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    if args.figures:
        plot_results(runs, args.figures)
    print(json.dumps({'verification_reussie': checks['passed'], 'cas': results['cases']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
