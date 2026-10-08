from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent / 'equations'
OUT.mkdir(parents=True, exist_ok=True)
EQUATIONS = {
 'state': r'$q=(G,s),\quad G\in SE(3),\quad s=(h,\theta,\phi,\rho),\quad \nu=(\xi_B,\dot{s})$',
 'dynamics': r'$M(q)\dot{\nu}+c(q,\nu)+g_q(q)+d(q,\nu)=B(q)u_{\mathrm{mech}}+J_c(q)^T\lambda_c$',
 'kinematics': r'$\dot{G}=G\widehat{\xi}_B,\qquad \xi_B=(\omega_B,v_B),\qquad \dot{s}=\nu_s$',
 'mech': r'$u_{\mathrm{mech}}=(F_h,\tau_A,\tau_W),\qquad u_{\mathrm{cmd}}=(a_{\mathrm{ref}},\theta_{\mathrm{ref}},\tau_{W,\mathrm{ref}})$',
 'centroid': r'$F=m(\ddot{p}_C+g e_3),\qquad M_O=\dot{H}_C+p_C\times F$',
 'moment': r'$x_Z=-\frac{M_{O,y}}{F_z},\qquad y_Z=\frac{M_{O,x}}{F_z},\qquad F_z>0$',
 'zmpx': r'$x_Z=x_C-\frac{z_C m\ddot{x}_C+\dot{H}_{C,y}}{m(g+\ddot{z}_C)}$',
 'zmpy': r'$y_Z=y_C-\frac{z_C m\ddot{y}_C-\dot{H}_{C,x}}{m(g+\ddot{z}_C)}$',
 'centermass': r'$m=\sum_i m_i,\qquad p_C=\frac{1}{m}\sum_i m_i p_{C,i}$',
 'momentum': r'$H_C=\sum_i\left[R_i I_i\omega_i^B+(p_{C,i}-p_C)\times m_i(\dot{p}_{C,i}-\dot{p}_C)\right]$',
 'frame': r'$p_Z^S=R_2(\psi)^T(p_Z^W-o_S^W),\qquad e_Z=p_Z^S-p_{Z,\mathrm{ref}}^S$',
 'stage': r'$\ell_{\mathrm{stab}}=e_Z^T Q_Z e_Z,\qquad Q_Z=\mathrm{diag}(a^{-2},b^{-2})$',
 'core': r'$p_{Z,\mathrm{ref}}^S=(0,0)^T\quad\Longrightarrow\quad\ell_{\mathrm{stab}}=\left(\frac{x_Z^S}{a}\right)^2+\left(\frac{y_Z^S}{b}\right)^2$',
 'cost': r'$J_{\mathrm{stab}}=\frac{1}{T}\int_0^T\ell_{\mathrm{stab}}(t)\,dt\;\simeq\;\frac{1}{N}\sum_{k=0}^{N-1}\ell_{\mathrm{stab},k}$',
 'bounds': r'$|x_Z^S|\leq a-\delta,\qquad |y_Z^S|\leq b-\delta,\qquad 0<\delta<\min(a,b)$',
 'opt': r'$\min_{u_{\mathrm{cmd}}(\cdot)} J_{\mathrm{stab}}\quad\mathrm{subject\ to\ dynamics,\ task,\ contact,\ and\ actuator\ constraints}$',
 'extension': r'$F_h=\frac{da}{dh}F_a,\qquad a_{\mathrm{meas}}=k_p V_p+b_p$',
}
for name, equation in EQUATIONS.items():
    fig = plt.figure(figsize=(11.5, 0.75), dpi=220)
    fig.text(.5, .5, equation, ha='center', va='center', fontsize=17, color='#123047')
    fig.savefig(OUT / (name+'.png'), transparent=True, bbox_inches='tight', pad_inches=.10)
    plt.close(fig)
print('Rendered', len(EQUATIONS), 'equations')
