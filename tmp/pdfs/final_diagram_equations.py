from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path(__file__).parent/'final_diagram_equations';out.mkdir(exist_ok=True)
eqs={
 'control':r'$\min_{u}\;J$',
 'marginx':r'$|x_Z|\leq a-\delta$',
 'marginy':r'$|y_Z|\leq b-\delta$',
 'force':r'$F=m(\ddot{p}_C+ge_3)$',
 'moment':r'$M_O=\dot{H}_C+p_C\times F$',
 'zmpx':r'$x_Z=-M_{O,y}/F_z$',
 'zmpy':r'$y_Z=M_{O,x}/F_z,\quad F_z>0$',
 'stage':r'$L=(x_Z/a)^2+(y_Z/b)^2$',
 'cost':r'$J=\frac{1}{N}\sum_{k=1}^{N}L_k$',
 'wrench':r'$F,\;M_O$',
 'xy':r'$x_Z,\;y_Z$',
 'u':r'$u$',
 'input':r'$u=\left[a_{\mathrm{ref}},\;\tau_A^T,\;\tau_{W,\mathrm{ref}}^T\right]^T$',
 'J':r'$J$'
}
for name,text in eqs.items():
 fig=plt.figure(figsize=(6,.6),dpi=240)
 artist=fig.text(.5,.5,text,fontsize=20,ha='center',va='center',color='black')
 fig.canvas.draw()
 bounds=artist.get_window_extent().expanded(1.08,1.18).transformed(fig.dpi_scale_trans.inverted())
 fig.savefig(out/(name+'.png'),bbox_inches=bounds,pad_inches=0,transparent=True)
 plt.close(fig)
