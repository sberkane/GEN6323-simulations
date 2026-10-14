#!/usr/bin/env python3
"""GEN6323 — même robot, cinématique ou dynamique (modèle nominal pédagogique).

Dépendances : Python 3, numpy, matplotlib. Depuis le dossier du script :
    python simulation_chapitre4.py
Les résultats CSV/JSON et la démonstration HTML sont écrits près de ce fichier;
les figures PDF/PNG vont dans son sous-répertoire figures. --output DOSSIER
redirige toutes ces sorties. Les pas vérifiés sont 0.002 s et 0.001 s.

Axes : x_B vers l'avant, y_B vers la gauche, θ positif antihoraire;
b est la voie complète; τ_R>τ_L produit une accélération angulaire positive.
On suppose le roulement sans glissement, l'adhérence suffisante, un sol plan,
le centre de masse au milieu de l'essieu et une commande de couple idéale.
Aucune dynamique électrique, batterie, courbe couple-vitesse, perte d'adhérence,
incertitude, bruit, retard numérique ou perturbation externe n'est simulé.

La dynamique utilise les inerties effectives m_e et J_e (incluant une éventuelle
inertie de rotation des roues/transmissions déjà ramenée aux coordonnées v,ω).
Les valeurs choisies sont illustratives, sans identification sur un vrai robot.
La comparaison isole la bande passante de vitesse et la saturation des couples;
elle ne prouve aucune stabilité globale et ne simule pas le glissement.
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass, asdict
import csv
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


@dataclass(frozen=True)
class Parameters:
    m_e: float = 12.0       # kg, inertie effective en translation
    J_e: float = 0.70       # kg m², inertie effective en lacet
    r: float = 0.10         # m, rayon de chaque roue
    b: float = 0.45         # m, voie complète
    c_v: float = 1.0        # N s/m, dissipation longitudinale
    c_w: float = 0.08       # N m s, dissipation en lacet
    T_v: float = 0.25       # s, constante de temps imposée par la boucle de vitesse
    T_w: float = 0.18       # s, constante de temps de lacet
    tau_max: float = 1.40   # N m, limite symétrique par roue
    k_x: float = 1.50       # s^-1, correction d'erreur longitudinale
    k_y: float = 1.20       # m^-2, correction d'erreur latérale
    k_theta: float = 2.00   # s^-1, correction d'erreur d'orientation
    A: float = 3.0          # m, amplitude x de la courbe de référence
    B: float = 1.6          # m, amplitude y de la courbe de référence

P = Parameters()
COLORS = {'ideal':'#1F5A94', 'lag':'#168C8C', 'limited':'#D95D39',
          'navy':'#102A43', 'gray':'#687684', 'gold':'#F2B705'}
LABELS = {'ideal':'Cinématique idéale', 'lag':'Dynamique, couples non bornés',
          'limited':'Dynamique, couples bornés'}
MODES = ('ideal', 'lag', 'limited')


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def reference(t, T, p=P):
    """x_d,y_d,θ_d,v_d,ω_d,dot(v_d),dot(ω_d), avec s=2πt/T."""
    s = 2*np.pi*np.asarray(t)/T
    k = 2*np.pi/T
    dx,dy = p.A*np.cos(s), 2*p.B*np.cos(2*s)
    ddx,ddy = -p.A*np.sin(s), -4*p.B*np.sin(2*s)
    dddx,dddy = -p.A*np.cos(s), -8*p.B*np.cos(2*s)
    D = dx*dx + dy*dy
    N = dx*ddy - dy*ddx
    Dp = 2*(dx*ddx + dy*ddy)
    Np = dx*dddy - dy*dddx
    return np.array([p.A*np.sin(s), p.B*np.sin(2*s), np.arctan2(dy,dx),
                     k*np.sqrt(D), k*N/D,
                     k*k*(dx*ddx+dy*ddy)/np.sqrt(D),
                     k*k*(Np*D-N*Dp)/(D*D)])


def wheel_torques(F, M, p=P):
    """Ordre gauche,droite. F=(τ_L+τ_R)/r; M=b(τ_R-τ_L)/(2r)."""
    return np.array([p.r*F/2-p.r*M/p.b, p.r*F/2+p.r*M/p.b])


def generalized_efforts(tau, p=P):
    return np.array([(tau[0]+tau[1])/p.r,
                     p.b*(tau[1]-tau[0])/(2*p.r)])


def command(state, ref, p=P):
    """Boucle externe identique dans les trois expériences.

    e_x,e_y sont les coordonnées CORPS du vecteur p_d-p.
    v_c=v_d cos(e_θ)+k_x e_x,
    ω_c=ω_d+k_y v_d e_y+k_θ sin(e_θ).
    Les sorties sont des consignes, pas des vitesses garanties.
    """
    dx,dy = ref[0]-state[0], ref[1]-state[1]
    c,s = np.cos(state[2]), np.sin(state[2])
    ex,ey = c*dx+s*dy, -s*dx+c*dy
    et = wrap(ref[2]-state[2])
    return np.array([ref[3]*np.cos(et)+p.k_x*ex,
                     ref[4]+p.k_y*ref[3]*ey+p.k_theta*np.sin(et)])


def control(t, state, T, mode, p=P):
    ref = reference(t,T,p)
    cmd = command(state,ref,p)
    if mode == 'ideal':
        return ref,cmd,np.zeros(2),np.zeros(2)
    # Compensation nominale de la dissipation; aucune anticipation d'accélération.
    F_req = p.c_v*state[3] + p.m_e*(cmd[0]-state[3])/p.T_v
    M_req = p.c_w*state[4] + p.J_e*(cmd[1]-state[4])/p.T_w
    tau_req = wheel_torques(F_req,M_req,p)
    tau = np.clip(tau_req,-p.tau_max,p.tau_max) if mode=='limited' else tau_req
    return ref,cmd,tau_req,tau


def rhs(t, state, T, mode, p=P):
    _,cmd,_,tau = control(t,state,T,mode,p)
    if mode == 'ideal':
        v,w = cmd
        return np.array([v*np.cos(state[2]),v*np.sin(state[2]),w])
    v,w = state[3:5]
    F,M = generalized_efforts(tau,p)
    return np.array([v*np.cos(state[2]),v*np.sin(state[2]),w,
                     (F-p.c_v*v)/p.m_e,(M-p.c_w*w)/p.J_e])


def simulate(T, mode, dt=0.002, p=P):
    # Tous démarrent exactement sur la référence, à sa vitesse initiale.
    n = int(round(T/dt)); dt=T/n
    t = np.linspace(0,T,n+1)
    z = np.empty((n+1,3 if mode=='ideal' else 5))
    z[0] = reference(0,T,p)[:z.shape[1]]
    for i in range(n):
        ti,zi = t[i],z[i]
        k1=rhs(ti,zi,T,mode,p)
        k2=rhs(ti+dt/2,zi+dt*k1/2,T,mode,p)
        k3=rhs(ti+dt/2,zi+dt*k2/2,T,mode,p)
        k4=rhs(ti+dt,zi+dt*k3,T,mode,p)
        z[i+1]=zi+dt*(k1+2*k2+2*k3+k4)/6
    ref=reference(t,T,p).T
    cmd=np.empty((n+1,2)); req=np.empty((n+1,2)); applied=np.empty((n+1,2))
    for i in range(n+1):
        _,cmd[i],req[i],applied[i]=control(t[i],z[i],T,mode,p)
    vel=cmd if mode=='ideal' else z[:,3:5]
    err=np.linalg.norm(z[:,:2]-ref[:,:2],axis=1)
    # Couples nécessaires à une réalisation EXACTE de la référence prescrite.
    req_ref=wheel_torques(p.m_e*ref[:,5]+p.c_v*ref[:,3],
                          p.J_e*ref[:,6]+p.c_w*ref[:,4],p).T
    return {'t':t,'T':T,'mode':mode,'state':z,'ref':ref,'cmd':cmd,'vel':vel,
            'tau_req':req,'tau':applied,'tau_ref':req_ref,'position_error':err}


def mean_time(y,t):
    return float(np.trapezoid(y,t)/(t[-1]-t[0]))


def metrics(run,p=P):
    t=run['t']; e=run['position_error']; vel=run['vel']; ref=run['ref']
    sat=np.any(np.abs(run['tau_req'])>p.tau_max+1e-12,axis=1)
    return {
        'duration_s':run['T'], 'mode':run['mode'],
        'position_rmse_m':np.sqrt(mean_time(e*e,t)),
        'position_max_error_m':float(np.max(e)),
        'position_final_error_m':float(e[-1]),
        'v_command_rmse_m_s':np.sqrt(mean_time((vel[:,0]-run['cmd'][:,0])**2,t)),
        'omega_command_rmse_rad_s':np.sqrt(mean_time((vel[:,1]-run['cmd'][:,1])**2,t)),
        'v_reference_rmse_m_s':np.sqrt(mean_time((vel[:,0]-ref[:,3])**2,t)),
        'omega_reference_rmse_rad_s':np.sqrt(mean_time((vel[:,1]-ref[:,4])**2,t)),
        'saturation_fraction':mean_time(sat.astype(float),t) if run['mode']=='limited' else 0.0,
        'nominal_required_peak_wheel_torque_Nm':float(np.max(np.abs(run['tau_ref']))),
        'requested_peak_wheel_torque_Nm':float(np.max(np.abs(run['tau_req']))) if run['mode']!='ideal' else None,
        'applied_peak_wheel_torque_Nm':float(np.max(np.abs(run['tau']))) if run['mode']!='ideal' else None,
        'peak_reference_v_m_s':float(np.max(ref[:,3])),
        'peak_reference_abs_omega_rad_s':float(np.max(np.abs(ref[:,4]))),
        'peak_reference_lateral_acceleration_m_s2':float(np.max(np.abs(ref[:,3]*ref[:,4]))),
    }


def configure_plots():
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.titlesize':13,
        'axes.labelsize':11,'xtick.labelsize':10,'ytick.labelsize':10,
        'axes.edgecolor':'#687684','axes.labelcolor':'#203040','text.color':'#203040',
        'xtick.color':'#687684','ytick.color':'#687684','axes.spines.top':False,
        'axes.spines.right':False,'grid.alpha':.20,'grid.color':'#687684',
        'legend.frameon':False,'figure.facecolor':'white','axes.facecolor':'white',
        'pdf.fonttype':42,'savefig.bbox':'tight'})


def save_fig(fig,path):
    fig.savefig(path.with_suffix('.pdf'))
    fig.savefig(path.with_suffix('.png'),dpi=170)
    plt.close(fig)


def plot_results(runs,figdir,p=P):
    configure_plots()
    # Une même géométrie parcourue à deux cadences; aspect égal indispensable.
    fig,axes=plt.subplots(1,2,figsize=(12.2,4.7),layout='constrained')
    for ax,T in zip(axes,(32.,8.)):
        for mode in ('limited','lag','ideal'):
            d=runs[(T,mode)]
            ax.plot(d['state'][:,0],d['state'][:,1],color=COLORS[mode],
                    lw=2.2 if mode!='ideal' else 1.8,ls='--' if mode=='ideal' else '-',label=LABELS[mode])
        ax.scatter([0],[0],s=35,c=COLORS['navy'],zorder=5)
        ax.annotate('Départ',xy=(0,0),xytext=(.25,-.52),fontsize=10,
                    arrowprops={'arrowstyle':'-','color':COLORS['gray']})
        d=runs[(T,'limited')]; m=metrics(d)
        ax.set(title=f"{'Lent' if T==32 else 'Rapide'} : T = {T:.0f} s",
               xlabel='x (m)',ylabel='y (m)',xlim=(-3.6,3.6),ylim=(-2.1,2.1))
        ax.set_aspect('equal'); ax.grid(True)
        ax.text(.02,.98,f"Erreur RMS, couples bornés : {100*m['position_rmse_m']:.1f} cm",
                va='top',transform=ax.transAxes,fontsize=10,
                bbox={'facecolor':'white','edgecolor':'none','alpha':.90})
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles[::-1],labels[::-1],loc='outside lower center',ncol=3,fontsize=10)
    save_fig(fig,figdir/'sim_trajectoires')

    fig,axes=plt.subplots(2,2,figsize=(12.2,6.0),sharex=True,layout='constrained')
    for j,T in enumerate((32.,8.)):
        d=runs[(T,'limited')]; s=d['t']/T
        for k in range(2):
            ax=axes[k,j]
            ax.plot(s,d['ref'][:,3+k],color=COLORS['ideal'],ls='--',lw=1.6,label=r'Référence $v_d,\,\omega_d$')
            ax.plot(s,d['cmd'][:,k],color=COLORS['gray'],ls=':',lw=1.9,label=r'Consigne $v_c,\,\omega_c$')
            ax.plot(s,d['vel'][:,k],color=COLORS['limited'],lw=2,label=r'Vitesse réelle $v,\,\omega$')
            ax.set_ylabel('v (m/s)' if k==0 else r'$\omega$ (rad/s)'); ax.grid(True)
        axes[0,j].set_title(f"{'Lent' if T==32 else 'Rapide'} : T = {T:.0f} s")
        axes[1,j].set_xlabel('Temps normalisé t / T')
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside lower center',ncol=3,fontsize=10)
    save_fig(fig,figdir/'sim_vitesses')

    fig,axes=plt.subplots(1,2,figsize=(12.2,3.8),sharey=True,layout='constrained')
    for ax,T in zip(axes,(32.,8.)):
        d=runs[(T,'limited')]; s=d['t']/T
        for j,side in enumerate(('L','R')):
            col=COLORS['ideal'] if j==0 else COLORS['limited']
            ax.plot(s,d['tau_req'][:,j],color=col,ls=':',lw=1.6,alpha=.8,label=rf'$\tau_{side}^\star$ demandé')
            ax.plot(s,d['tau'][:,j],color=col,lw=2,label=rf'$\tau_{side}$ appliqué')
        for v in (-p.tau_max,p.tau_max): ax.axhline(v,color=COLORS['gray'],lw=1,ls='--')
        ax.fill_between(s,-p.tau_max,p.tau_max,color=COLORS['teal'] if 'teal' in COLORS else '#168C8C',alpha=.06)
        m=metrics(d)
        ax.set(title=f"T = {T:.0f} s — saturation : {100*m['saturation_fraction']:.1f} % du temps",
               xlabel='Temps normalisé t / T',ylabel='Couple de roue (N·m)')
        ax.grid(True)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside lower center',ncol=4,fontsize=9)
    save_fig(fig,figdir/'sim_couples')

    fig,axes=plt.subplots(1,2,figsize=(12.2,3.8),layout='constrained')
    for ax,T in zip(axes,(32.,8.)):
        for mode in MODES:
            d=runs[(T,mode)]
            ax.plot(d['t']/T,100*d['position_error'],color=COLORS[mode],lw=2,
                    ls='--' if mode=='ideal' else '-',label=LABELS[mode])
        ax.set(title=f"{'Lent' if T==32 else 'Rapide'} : T = {T:.0f} s",
               xlabel='Temps normalisé t / T',ylabel='Erreur de position (cm)')
        ax.grid(True)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside lower center',ncol=3,fontsize=10)
    save_fig(fig,figdir/'sim_erreurs')

    fig,axes=plt.subplots(1,2,figsize=(12.2,3.8),sharey=True,layout='constrained')
    for ax,T in zip(axes,(32.,8.)):
        d=runs[(T,'limited')]; s=d['t']/T
        for j,side in enumerate(('L','R')):
            ax.plot(s,d['tau_ref'][:,j],color=COLORS['ideal'] if j==0 else COLORS['limited'],
                    lw=2,label=rf'$\tau_{{{side},d}}$ requis pour suivre exactement')
        for v in (-p.tau_max,p.tau_max): ax.axhline(v,color=COLORS['gray'],lw=1,ls='--')
        ax.fill_between(s,-p.tau_max,p.tau_max,color='#168C8C',alpha=.07)
        ax.set(title=f"T = {T:.0f} s — pic requis : {np.max(np.abs(d['tau_ref'])):.2f} N·m",
               xlabel='Temps normalisé t / T',ylabel='Couple nominal requis (N·m)')
        ax.grid(True)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside lower center',ncol=2,fontsize=10)
    save_fig(fig,figdir/'sim_faisabilite')


def write_csv(run,directory):
    path=directory/f"chapitre4_T{run['T']:.0f}_{run['mode']}.csv"
    heading=['t_s','t_sur_T','x_m','y_m','theta_rad','v_m_s','omega_rad_s',
             'x_d_m','y_d_m','theta_d_rad','v_d_m_s','omega_d_rad_s',
             'a_d_m_s2','alpha_d_rad_s2','v_c_m_s','omega_c_rad_s',
             'tau_L_requested_Nm','tau_R_requested_Nm','tau_L_applied_Nm','tau_R_applied_Nm',
             'tau_L_reference_Nm','tau_R_reference_Nm','position_error_m']
    table=np.column_stack([run['t'],run['t']/run['T'],run['state'][:,:3],run['vel'],
                          run['ref'],run['cmd'],run['tau_req'],run['tau'],
                          run['tau_ref'],run['position_error']])
    if run['mode']=='ideal':
        table[:,16:20]=np.nan  # Les couples ne sont pas des variables du modèle cinématique.
    np.savetxt(path,table,delimiter=',',header=','.join(heading),comments='',fmt='%.10g')


def validate(runs,dt,p=P):
    checks={}
    rng=np.random.default_rng(6323)
    F,M=rng.normal(size=(2,1000))
    tau=wheel_torques(F,M,p)
    recovered=generalized_efforts(tau,p)
    checks['wheel_effort_inverse_max_abs_error']=float(np.max(np.abs(recovered-np.array([F,M]))))
    v,w=rng.normal(size=(2,1000)); rates=np.array([(v-p.b*w/2)/p.r,(v+p.b*w/2)/p.r])
    checks['power_identity_max_abs_error_W']=float(np.max(np.abs(np.sum(tau*rates,axis=0)-(F*v+M*w))))
    sample=rng.uniform(.01,7.99,1000); h=1e-5
    plus=reference(sample+h,8,p); minus=reference(sample-h,8,p); mid=reference(sample,8,p)
    checks['reference_dv_finite_difference_max_abs_error']=float(np.max(np.abs((plus[3]-minus[3])/(2*h)-mid[5])))
    checks['reference_domega_finite_difference_max_abs_error']=float(np.max(np.abs((plus[4]-minus[4])/(2*h)-mid[6])))
    sample_s=np.linspace(0,2*np.pi,10001)
    min_tangent_sq=float(np.min((p.A*np.cos(sample_s))**2+(2*p.B*np.cos(2*sample_s))**2))
    checks['reference_min_tangent_squared_m2']=min_tangent_sq
    checks['ideal_max_position_error_m']=max(metrics(runs[(T,'ideal')])['position_max_error_m'] for T in (32.,8.))
    algebra_error=0.0
    for _ in range(1000):
        z=rng.normal(size=5); tt=float(rng.uniform(0,8))
        _,cmd,_,_=control(tt,z,8,'lag',p)
        dz=rhs(tt,z,8,'lag',p)
        algebra_error=max(algebra_error,float(np.max(np.abs(dz[3:]-np.array([(cmd[0]-z[3])/p.T_v,(cmd[1]-z[4])/p.T_w])))))
    checks['unsaturated_inner_loop_identity_max_abs_error']=algebra_error
    # Pas divisé par 2, cas le plus difficile, avec tous les points communs.
    fine=simulate(8.,'limited',dt/2,p); coarse=runs[(8.,'limited')]
    checks['halved_step_max_position_discrepancy_m']=float(np.max(np.linalg.norm(fine['state'][::2,:2]-coarse['state'][:,:2],axis=1)))
    checks['halved_step_position_rmse_difference_m']=abs(metrics(fine)['position_rmse_m']-metrics(coarse)['position_rmse_m'])
    checks['halved_step_max_speed_discrepancy_m_s']=float(np.max(np.abs(fine['vel'][::2,0]-coarse['vel'][:,0])))
    checks['applied_torque_limit_excess_Nm']=max(float(np.max(np.abs(runs[(T,'limited')]['tau']))-p.tau_max) for T in (32.,8.))
    checks['pass']=(checks['wheel_effort_inverse_max_abs_error']<1e-12
                    and checks['power_identity_max_abs_error_W']<1e-12
                    and checks['reference_dv_finite_difference_max_abs_error']<1e-7
                    and checks['reference_domega_finite_difference_max_abs_error']<1e-7
                    and min_tangent_sq>0
                    and checks['ideal_max_position_error_m']<1e-6
                    and algebra_error<1e-10
                    and checks['halved_step_max_position_discrepancy_m']<2e-4
                    and checks['applied_torque_limit_excess_Nm']<1e-12)
    assert checks['pass'],checks
    return checks


def validated_step(value):
    """Pas pris en charge par les validations et l'échantillonnage exporté."""
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
    args=parser.parse_args()
    here=Path(__file__).resolve().parent
    out=args.output or here
    figdir=out/'figures'
    figdir.mkdir(parents=True,exist_ok=True)
    runs={}
    for T in (32.,8.):
        for mode in MODES:
            print(f'Simulation T={T:.0f}s, {mode}',flush=True)
            runs[(T,mode)]=simulate(T,mode,args.dt)
            write_csv(runs[(T,mode)],out)
    checks=validate(runs,args.dt)
    result={'parameters':asdict(P),'dt_s':args.dt,'initialization':'Pose et vitesses initiales exactement sur la référence.',
            'metrics':[metrics(d) for d in runs.values()], 'validation':checks}
    (out/'chapitre4_resultats.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    plot_results(runs,figdir)
    # Données compactes pour une animation HTML autonome, interpolation linéaire à l'écran.
    animation={str(int(T)):{mode:{'t':r['t'][::10].round(5).tolist(),
                  'pose':r['state'][::10,:3].round(7).tolist(),
                  'vel':r['vel'][::10].round(7).tolist(),
                  'cmd':r['cmd'][::10].round(7).tolist(),
                  'tau':r['tau'][::10].round(7).tolist(),
                  'ref':r['ref'][::10,:5].round(7).tolist(),
                  'metrics':metrics(r)} for mode in MODES for r in [runs[(T,mode)]]} for T in (32.,8.)}
    (out/'chapitre4_animation_data.json').write_text(json.dumps(animation,separators=(',',':'),ensure_ascii=False),encoding='utf-8')
    template=here/'chapitre4_animation_template.html'
    if template.exists():
        (out/'chapitre4_simulation.html').write_text(template.read_text(encoding='utf-8').replace('__SIM_DATA__',json.dumps(animation,separators=(',',':'),ensure_ascii=False)),encoding='utf-8')
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__':
    main()
