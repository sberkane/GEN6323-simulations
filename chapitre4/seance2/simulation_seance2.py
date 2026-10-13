#!/usr/bin/env python3
"""GEN6323 — commande de position, boucles P/PI/PID et anti-emballement.

Exécuter : python simulation_seance2.py
Dépendances : Python 3, NumPy, Matplotlib. Paramètres pédagogiques synthétiques.
Les équations sont intégrées par RK4 à pas fixe; un second calcul à dt/2 vérifie
la convergence. Les figures PDF vectorielles et PNG vont dans le dossier
figures voisin du script; le JSON est écrit près du script. --output DOSSIER
redirige ces sorties. Les pas vérifiés sont 0.002 s et 0.001 s.

Conventions : x_B en avant, y_B à gauche, theta positif antihoraire. Les couples
sont des couples moteurs équivalents ramenés aux roues; m_e,J_e incluent les
inerties tournantes. r est le rayon, b la voie complète. Les références et
consignes (suffixe c) sont distinguées des vitesses réellement simulées.
Le modèle suppose roulement pur, adhérence suffisante et estimation parfaite.
Pas de dynamique électrique, bruit, retard d'échantillonnage ni glissement.
La régulation vise une position fixe, sans imposer l'orientation finale.
Une simulation de la cascade ne constitue pas une preuve de stabilité globale.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

COLORS = {'navy':'#102A43', 'blue':'#1F5A94', 'teal':'#168C8C',
          'orange':'#D95D39', 'gray':'#687684', 'gold':'#F2B705'}
P = dict(m_e=12., J_e=.7, c_v=1., c_omega=.08, r=.1, b=.45,
         goal=[2.,1.], k_rho=.6, k_alpha=1.5,
         k_Pv=4., k_Iv=4., k_Pomega=6., k_Iomega=9.,
         k_D=.4, T_f=.05, T_aw=.3, tau_max_cascade=1.4,
         tau_max_windup=.4, epsilon_goal=1e-9)


def wrap(a):
    """Angle principal par atan2; la coupure +/-pi n'est pas un état continu."""
    return np.arctan2(np.sin(a), np.cos(a))


def point_control(q):
    delta = np.asarray(P['goal']) - q[:2]
    rho = np.linalg.norm(delta)
    if rho < P['epsilon_goal']:
        # Protection numérique uniquement : aucun arrêt à distance finie.
        return np.zeros(4)
    alpha = wrap(np.arctan2(delta[1], delta[0]) - q[2])
    v_c = P['k_rho'] * rho * np.cos(alpha)  # signé : recul autorisé
    omega_c = P['k_alpha'] * alpha + P['k_rho'] * np.sin(alpha)*np.cos(alpha)
    return np.array([v_c, omega_c, rho, alpha])


def ideal_rhs(t, q):
    v, w = point_control(q)[:2]
    return np.array([v*np.cos(q[2]), v*np.sin(q[2]), w])


def allocation(F, M):
    return np.array([P['r']*F/2-P['r']*M/P['b'],
                     P['r']*F/2+P['r']*M/P['b']])


def efforts(tau):
    return np.array([(tau[0]+tau[1])/P['r'],
                     P['b']*(tau[1]-tau[0])/(2*P['r'])])


def rk4(rhs, y0, horizon, dt):
    steps = int(round(horizon / dt))
    if abs(steps*dt-horizon) > 1e-10:
        raise ValueError('La durée doit être un multiple du pas.')
    t = np.arange(steps+1)*dt
    y = np.empty((steps+1,len(y0)), dtype=float)
    y[0] = y0
    for i in range(steps):
        ti, yi = t[i], y[i]
        a = rhs(ti,yi)
        b = rhs(ti+dt/2, yi+dt*a/2)
        c = rhs(ti+dt/2, yi+dt*b/2)
        d = rhs(ti+dt, yi+dt*c)
        y[i+1] = yi+dt*(a+2*b+2*c+d)/6
    if not np.all(np.isfinite(y)):
        raise RuntimeError('État non fini dans la simulation.')
    return t, y


def inner_signals(y, mode, disturbance):
    v, integ, z = y
    error = 1-v
    ki = P['k_Iv'] if mode in ('PI','PID') else 0.
    kd = P['k_D'] if mode == 'PID' else 0.
    force = P['c_v']*v+P['m_e']*(P['k_Pv']*error+ki*integ-kd*z)
    dv = (force-P['c_v']*v+disturbance)/P['m_e']
    return np.array([dv, error if ki else 0., (dv-z)/P['T_f'] if kd else 0.]), force


def inner_run(mode, dt, horizon=20.):
    # Intégration segmentée : l'échelon de charge ne contamine pas le dernier
    # stade RK4 de l'intervalle situé juste avant 5 s.
    t0, y0 = rk4(lambda t,y: inner_signals(y,mode,0.)[0], [0.,0.,0.], 5., dt)
    t1, y1 = rk4(lambda t,y: inner_signals(y,mode,-3.)[0], y0[-1], horizon-5., dt)
    t = np.r_[t0,5+t1[1:]]
    y = np.vstack((y0,y1[1:]))
    force = np.array([inner_signals(yi,mode,-3. if ti>=5. else 0.)[1] for ti,yi in zip(t,y)])
    return dict(t=t,y=y,force=force)


def windup_signals(y, anti):
    v, integ = y
    error = 1-v
    force_star = P['c_v']*v+P['m_e']*(P['k_Pv']*error+P['k_Iv']*integ)
    bound = 2*P['tau_max_windup']/P['r']
    force = np.clip(force_star,-bound,bound)
    di = error
    if anti:
        di += (force-force_star)/(P['m_e']*P['k_Iv']*P['T_aw'])
    dv = (force-P['c_v']*v)/P['m_e']
    return np.array([dv,di]), np.array([force_star,force])


def windup_run(anti,dt):
    t,y = rk4(lambda t,y: windup_signals(y,anti)[0], [0.,0.],12.,dt)
    force = np.array([windup_signals(yi,anti)[1] for yi in y])
    return dict(t=t,y=y,force=force)


def cascade_signals(y):
    x, yy, theta, v, omega, iv, iw = y
    vc, wc, rho, alpha = point_control(y[:3])
    ev, ew = vc-v,wc-omega
    Fstar = P['c_v']*v+P['m_e']*(P['k_Pv']*ev+P['k_Iv']*iv)
    Mstar = P['c_omega']*omega+P['J_e']*(P['k_Pomega']*ew+P['k_Iomega']*iw)
    torque_star = allocation(Fstar,Mstar)
    torque = np.clip(torque_star,-P['tau_max_cascade'],P['tau_max_cascade'])
    F,M = efforts(torque)
    div = ev+(F-Fstar)/(P['m_e']*P['k_Iv']*P['T_aw'])
    diw = ew+(M-Mstar)/(P['J_e']*P['k_Iomega']*P['T_aw'])
    rhs = np.array([v*np.cos(theta),v*np.sin(theta),omega,
                    (F-P['c_v']*v)/P['m_e'],
                    (M-P['c_omega']*omega)/P['J_e'],div,diw])
    return rhs,np.r_[vc,wc,rho,alpha,torque_star,torque,Fstar,Mstar,F,M]


def cascade_run(dt):
    t,y = rk4(lambda t,y: cascade_signals(y)[0],[0.]*7,12.,dt)
    sig=np.array([cascade_signals(yi)[1] for yi in y])
    return dict(t=t,y=y,sig=sig)


def settle_time(t,error,tol):
    bad = np.flatnonzero(np.abs(error)>tol)
    if not len(bad): return float(t[0])
    if bad[-1]==len(t)-1: return None
    return float(t[bad[-1]+1])


def basic_metrics(t,v):
    return dict(v_final_m_s=float(v[-1]),
                overshoot_percent=float(100*max(0.,np.max(v)-1.)),
                time_peak_s=float(t[np.argmax(v)]),
                settling_2percent_s=settle_time(t,v-1.,.02))


def sim_all(dt):
    ideal=[]
    for angle in (0.,np.pi/2,-2.):
        t,y=rk4(ideal_rhs,[0.,0.,angle],12.,dt)
        sig=np.array([point_control(yi) for yi in y])
        ideal.append(dict(t=t,y=y,sig=sig,initial_heading=angle))
    return dict(ideal=ideal,
                inner={mode:inner_run(mode,dt) for mode in ('P','PI','PID')},
                windup={mode:windup_run(mode=='anti',dt) for mode in ('sans','anti')},
                cascade=cascade_run(dt))


def verification(data,dt):
    fine=sim_all(dt/2)
    checked={}
    for i,(a,b) in enumerate(zip(data['ideal'],fine['ideal'])):
        checked['ideal_'+str(i)+'_position_max_m']=float(np.linalg.norm(a['y'][:,:2]-b['y'][::2,:2],axis=1).max())
        # Vérification indépendante : dot alpha=-k_alpha*alpha et rho_dot=-k_rho*rho*cos²alpha.
        predicted=a['sig'][0,3]*np.exp(-P['k_alpha']*a['t'])
        checked['ideal_'+str(i)+'_alpha_analytical_max_rad']=float(np.max(np.abs(predicted-a['sig'][:,3])))
        assert checked['ideal_'+str(i)+'_alpha_analytical_max_rad']<1e-7
    for mode in ('P','PI','PID'):
        checked['inner_'+mode+'_speed_max_m_s']=float(np.max(np.abs(data['inner'][mode]['y'][:,0]-fine['inner'][mode]['y'][::2,0])))
    for mode in ('sans','anti'):
        checked['windup_'+mode+'_speed_max_m_s']=float(np.max(np.abs(data['windup'][mode]['y'][:,0]-fine['windup'][mode]['y'][::2,0])))
    checked['cascade_position_max_m']=float(np.linalg.norm(data['cascade']['y'][:,:2]-fine['cascade']['y'][::2,:2],axis=1).max())
    checked['cascade_speed_max_m_s']=float(np.max(np.abs(data['cascade']['y'][:,3]-fine['cascade']['y'][::2,3])))
    checked['cascade_angular_speed_max_rad_s']=float(np.max(np.abs(data['cascade']['y'][:,4]-fine['cascade']['y'][::2,4])))
    p=data['inner']['P']; t=p['t']; analytic=1-np.exp(-P['k_Pv']*t)
    after=t>=5.
    analytic[after]-=3/(P['m_e']*P['k_Pv'])*(1-np.exp(-P['k_Pv']*(t[after]-5)))
    checked['P_full_analytical_speed_max_m_s']=float(np.max(np.abs(analytic-p['y'][:,0])))
    assert checked['P_full_analytical_speed_max_m_s']<1e-8
    rng=np.random.default_rng(6323)
    allocation_error=power_error=0.
    for F,M,v,w in rng.normal(size=(1000,4)):
        torque=allocation(F,M)
        allocation_error=max(allocation_error,float(np.max(np.abs(efforts(torque)-[F,M]))))
        wheel_speed=np.array([v-P['b']*w/2,v+P['b']*w/2])/P['r']
        power_error=max(power_error,float(abs(torque@wheel_speed-(F*v+M*w))))
    checked['allocation_inverse_max_abs']=allocation_error
    checked['power_identity_max_abs_W']=power_error
    assert allocation_error<1e-12 and power_error<1e-12
    assert checked['cascade_position_max_m']<1e-4
    assert max(v for k,v in checked.items() if 'speed_max' in k)<1e-3
    return checked


def metrics(data,dt):
    out=dict(parameters=P,units=dict(m_e='kg',J_e='kg m²',c_v='N s/m',c_omega='N m s',r='m',b='m',
              goal='m',k_rho='s^-1',k_alpha='s^-1',k_Pv='s^-1',k_Iv='s^-2',
              k_Pomega='s^-1',k_Iomega='s^-2',k_D='sans dimension',T_f='s',T_aw='s',
              tau_max_cascade='N m',tau_max_windup='N m',epsilon_goal='m'),
             integration=dict(method='RK4 pas fixe, échelon de charge segmenté à 5 s',dt_s=dt,dt_check_s=dt/2),
             assumptions=['Paramètres synthétiques','Roulement pur, adhérence suffisante','Mesure parfaite',
                          'Couples équivalents aux roues','Aucune preuve globale pour la cascade simulée'])
    out['ideal']=[]
    for item in data['ideal']:
        out['ideal'].append(dict(initial_pose=[0,0,float(item['initial_heading'])],
             goal=P['goal'],horizon_s=12.,rho_final_m=float(item['sig'][-1,2]),
             alpha_final_rad=float(item['sig'][-1,3]),
             min_commanded_speed_m_s=float(item['sig'][:,0].min()),
             reverse_fraction=float(np.mean(item['sig'][:-1,0]<0)),
             target_guard_activated=bool(np.any(item['sig'][:,2]<P['epsilon_goal']))))
    out['inner']=dict(reference_m_s=1.,initial_speed_m_s=0.,horizon_s=20.,
                       disturbance=dict(time_s=5.,force_N=-3.),
                       torque_saturation=False, derivative_on='mesure v, filtrée; z ≈ dot v',
                       predicted_P_steady_error_m_s=3/(P['m_e']*P['k_Pv']))
    for mode,item in data['inner'].items():
        pre=item['t']<5.; post=item['t']>=5.
        out['inner'][mode]=dict(**basic_metrics(item['t'][pre],item['y'][pre,0]),
             speed_final_20s_m_s=float(item['y'][-1,0]),
             error_final_20s_m_s=float(1-item['y'][-1,0]),
             min_speed_after_load_m_s=float(item['y'][post,0].min()),
             steady_force_N=float(item['force'][-1]))
    out['windup']=dict(reference_m_s=1.,horizon_s=12.,disturbance_N=0.,force_limit_N=8.)
    for mode,item in data['windup'].items():
        sat=np.abs(item['force'][:,0]-item['force'][:,1])>1e-9
        out['windup'][mode]=dict(**basic_metrics(item['t'],item['y'][:,0]),
             integral_final_m=float(item['y'][-1,1]),
             saturation_duration_s=float(np.sum(sat[:-1])*dt),
             requested_wheel_torque_peak_Nm=float(np.max(np.abs(item['force'][:,0]))*P['r']/2),
             applied_wheel_torque_peak_Nm=float(np.max(np.abs(item['force'][:,1]))*P['r']/2))
    ca=data['cascade']; saturated=np.any(np.abs(ca['sig'][:,4:6]-ca['sig'][:,6:8])>1e-9,axis=1)
    out['cascade']=dict(initial_state=[0]*7,horizon_s=12.,rho_final_m=float(ca['sig'][-1,2]),
            rho_min_m=float(ca['sig'][:,2].min()),alpha_final_rad=float(ca['sig'][-1,3]),
            max_wrapped_alpha_step_rad=float(np.max(np.abs(np.diff(ca['sig'][:,3])))),
            target_guard_activated=bool(np.any(ca['sig'][:,2]<P['epsilon_goal'])),
            saturation_duration_s=float(np.sum(saturated[:-1])*dt),
            requested_wheel_torque_peak_Nm=float(np.max(np.abs(ca['sig'][:,4:6]))),
            applied_wheel_torque_peak_Nm=float(np.max(np.abs(ca['sig'][:,6:8]))),
            velocity_rms_error_m_s=float(np.sqrt(np.mean((ca['sig'][:,0]-ca['y'][:,3])**2))),
            angular_velocity_rms_error_rad_s=float(np.sqrt(np.mean((ca['sig'][:,1]-ca['y'][:,4])**2))))
    return out


def plot_style():
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7.4,'axes.titlesize':8.2,
       'axes.titleweight':'bold','axes.labelsize':7.4,'xtick.labelsize':6.8,'ytick.labelsize':6.8,
       'legend.fontsize':6.9,'axes.labelcolor':COLORS['navy'],'text.color':COLORS['navy'],
       'axes.edgecolor':'#AAB3BB','xtick.color':COLORS['navy'],'ytick.color':COLORS['navy'],
       'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,
       'grid.color':'#E3E8ED','grid.linewidth':.45,'lines.linewidth':1.2,
       'axes.axisbelow':True,'pdf.fonttype':42,'savefig.facecolor':'white'})


def new_fig(n):
    fig,axes=plt.subplots(1,n,figsize=(13/2.54,5.3/2.54),layout=None)
    fig.subplots_adjust(left=.082,right=.987,bottom=.24,top=.78,wspace=.47 if n==3 else .31)
    return fig,axes


def save(fig,path):
    fig.savefig(path.with_suffix('.pdf'))
    fig.savefig(path.with_suffix('.png'),dpi=240)
    plt.close(fig)


def footprint(ax,q,color):
    # Plan du châssis schématique, centré sur B; x_B orienté selon theta.
    shape=np.array([[-.08,-.05],[.08,-.05],[.08,.05],[-.08,.05]])
    ct,st=np.cos(q[2]),np.sin(q[2]); rot=np.array([[ct,-st],[st,ct]])
    ax.add_patch(Polygon(shape@rot.T+q[:2],facecolor='white',edgecolor=color,lw=.75,zorder=4))
    ax.annotate('',xy=q[:2]+.19*np.array([ct,st]),xytext=q[:2],
                arrowprops=dict(arrowstyle='-|>',color=color,lw=.8,mutation_scale=6),zorder=5)


def xy_format(ax):
    ax.plot(*P['goal'],marker='*',color=COLORS['navy'],ms=7,zorder=7)
    ax.set_xlabel('$x$ (m)');ax.set_ylabel('$y$ (m)',labelpad=1)
    ax.set_aspect('equal',adjustable='box');ax.set_xlim(-.2,2.25);ax.set_ylim(-.24,1.32)
    ax.set_xticks([0,1,2]);ax.set_yticks([0,.5,1.])


def figures(data,outdir):
    plot_style();outdir.mkdir(parents=True,exist_ok=True)
    fig,ax=new_fig(3)
    colors=[COLORS['blue'],COLORS['teal'],COLORS['orange']]
    labels=['$\\theta(0)=0$','$\\theta(0)=\\pi/2$','$\\theta(0)=-2$ rad']
    lines=[]
    for d,color,label in zip(data['ideal'],colors,labels):
        line,=ax[0].plot(d['y'][:,0],d['y'][:,1],color=color,label=label);lines.append(line)
        ax[1].semilogy(d['t'],d['sig'][:,2],color=color)
        ax[2].plot(d['t'],d['sig'][:,3],color=color)
    # Les empreintes de la trajectoire bleue évitent de superposer six châssis.
    footprint(ax[0],data['ideal'][0]['y'][0],colors[0]);footprint(ax[0],data['ideal'][0]['y'][-1],colors[0])
    xy_format(ax[0]);ax[0].set_title('Trajectoires')
    ax[1].set(title='Distance au but',xlabel='$t$ (s)',ylabel='$\\rho$ (m)',xlim=(0,12));ax[1].set_xticks([0,6,12])
    ax[2].set(title='Erreur de direction',xlabel='$t$ (s)',ylabel='$\\alpha$ (rad)',xlim=(0,12));ax[2].set_xticks([0,6,12]);ax[2].set_yticks([-1,0,1,2])
    fig.legend(lines,labels,loc='upper center',bbox_to_anchor=(.51,1.025),ncol=3,frameon=False,columnspacing=1.25,handlelength=1.8)
    save(fig,outdir/'seance2_regulation_position')

    fig,ax=new_fig(2);lines=[]
    for mode,color in zip(('P','PI','PID'),colors):
        d=data['inner'][mode]
        line,=ax[0].plot(d['t'],d['y'][:,0],color=color,label=mode);lines.append(line)
        ax[1].plot(d['t'],d['force'],color=color)
    ax[0].axhline(1,color=COLORS['gray'],lw=.7,ls='--')
    for a in ax:
        a.axvline(5,color=COLORS['gray'],lw=.8,ls=':');a.set_xlim(0,20);a.set_xlabel('$t$ (s)');a.set_xticks([0,5,10,15,20])
    ax[0].set(title='Vitesse réelle',ylabel='$v$ (m/s)',ylim=(-.03,1.27))
    ax[1].set(title='Force appliquée',ylabel='$F_a$ (N)')
    ax[0].annotate('charge à 5 s',xy=(5,.14),xytext=(6.6,.22),fontsize=6.6,color=COLORS['gray'],arrowprops=dict(arrowstyle='-',lw=.5,color=COLORS['gray']))
    fig.legend(lines,['P','PI','PID filtré'],loc='upper center',bbox_to_anchor=(.53,1.025),ncol=3,frameon=False,handlelength=2.)
    save(fig,outdir/'seance2_comparaison_pid')

    fig,ax=new_fig(2);legend_handles=[]
    for mode,color,label in [('sans',COLORS['orange'],'Sans anti-emballement'),('anti',COLORS['teal'],'Avec rétro-calcul')]:
        d=data['windup'][mode]
        line,=ax[0].plot(d['t'],d['y'][:,0],color=color,label=label);legend_handles.append(line)
        ax[1].plot(d['t'],d['force'][:,0]*P['r']/2,color=color,ls='--',lw=.95)
        ax[1].plot(d['t'],d['force'][:,1]*P['r']/2,color=color,lw=1.2)
    ax[0].axhline(1,color=COLORS['gray'],lw=.7,ls=':')
    for a in ax:
        a.set_xlim(0,12);a.set_xlabel('$t$ (s)');a.set_xticks([0,3,6,9,12])
    ax[1].axhline(.4,color=COLORS['gray'],lw=.7,ls=':');ax[1].axhline(-.4,color=COLORS['gray'],lw=.7,ls=':')
    ax[0].set(title='Vitesse réelle',ylabel='$v$ (m/s)')
    ax[1].set(title='Couple par roue',ylabel='$\\tau$ (N m)')
    fig.legend(legend_handles,[h.get_label() for h in legend_handles],loc='upper center',bbox_to_anchor=(.54,1.025),ncol=2,frameon=False,handlelength=2.,columnspacing=1.3)
    fig.text(.75,.022,'— appliqué   -- demandé',ha='center',fontsize=7.)
    save(fig,outdir/'seance2_anti_emballement')

    fig,ax=new_fig(3);ca=data['cascade'];ideal=data['ideal'][0]
    ax[0].plot(ideal['y'][:,0],ideal['y'][:,1],color=COLORS['blue'],ls='--',label='Cinématique idéale')
    ax[0].plot(ca['y'][:,0],ca['y'][:,1],color=COLORS['orange'],label='Cascade dynamique')
    footprint(ax[0],ca['y'][0,:3],COLORS['orange']);footprint(ax[0],ca['y'][-1,:3],COLORS['orange'])
    xy_format(ax[0]);ax[0].set_title('Position du robot')
    for i,col in [(1,3),(2,4)]:
        ax[i].plot(ca['t'],ca['sig'][:,i-1],color=COLORS['teal'],ls='--',label='Consigne')
        ax[i].plot(ca['t'],ca['y'][:,col],color=COLORS['orange'],label='Vitesse réelle')
        ax[i].set_xlim(0,12);ax[i].set_xticks([0,6,12]);ax[i].set_xlabel('$t$ (s)')
    ax[1].set(title='Translation',ylabel='$v_c, v$ (m/s)')
    ax[2].set(title='Rotation',ylabel='$\\omega_c, \\omega$ (rad/s)')
    h0,l0=ax[0].get_legend_handles_labels();h1,l1=ax[1].get_legend_handles_labels()
    fig.legend(h0+h1,l0+l1,loc='upper center',bbox_to_anchor=(.53,1.01),ncol=4,frameon=False,handlelength=1.4,columnspacing=.85,fontsize=6.4)
    save(fig,outdir/'seance2_cascade_position')


def validated_step(value):
    """Pas divisant les horizons et compatible avec les validations numériques."""
    try:
        step = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError('Le pas doit être un nombre en secondes.') from None
    if not math.isfinite(step) or step <= 0:
        raise argparse.ArgumentTypeError('Le pas doit être fini et strictement positif.')
    if step not in (.002, .001):
        raise argparse.ArgumentTypeError('Pas pris en charge : 0.002 ou 0.001 seconde.')
    return step


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dt',type=validated_step,default=.002,metavar='{0.002,0.001}',
                        help='Pas RK4 en secondes (défaut : 0.002).')
    parser.add_argument('--output',type=Path,metavar='DOSSIER',
                        help='Dossier des résultats et figures (défaut : près du script).')
    parser.add_argument('--figures',type=Path,metavar='DOSSIER',
                        help='Dossier des figures; prioritaire sur --output.')
    parser.add_argument('--results',type=Path,metavar='FICHIER',
                        help='Fichier JSON; prioritaire sur --output.')
    args=parser.parse_args()
    here=Path(__file__).resolve().parent
    out=args.output or here
    figure_dir=args.figures or out/'figures'
    result_path=args.results or out/'resultats_seance2.json'
    data=sim_all(args.dt)
    result=metrics(data,args.dt)
    result['verification']=verification(data,args.dt)
    figures(data,figure_dir)
    result_path.parent.mkdir(parents=True,exist_ok=True)
    result_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
