from pathlib import Path
from xml.sax.saxutils import escape
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.colors import HexColor, white
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import Paragraph, Table, TableStyle
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
import pypdfium2 as pdfium
from pypdf import PdfReader

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output/pdf'
OUT.mkdir(parents=True, exist_ok=True)
EQ = Path(__file__).parent / 'equations'
PDF = OUT / 'mobile_manipulator_zmp_stability.pdf'
ROBOT = Path('C:/Users/oviha/AppData/Local/Temp/codex-clipboard-d7b50df4-0503-427f-ae39-cca78809734f.png')
NAVY = HexColor('#123047'); TEAL = HexColor('#087e8b'); INK = HexColor('#243746')
LIGHT = HexColor('#eef5f8'); RULE = HexColor('#ccdce4')
style = ParagraphStyle('body', fontName='Helvetica', fontSize=10.1, leading=14.0, textColor=INK)
pdfmetrics.registerFont(TTFont('ArialSymbols','C:/Windows/Fonts/arial.ttf'))
small = ParagraphStyle('small', parent=style, fontSize=8.8, leading=11.8)
c = canvas.Canvas(str(PDF), pagesize=A4)
c.setTitle('ZMP-based stability formulation for a mobile manipulator')
c.setAuthor('Mobile manipulator research note')

def header(title, subtitle, page, size=A4):
    global W,H,y
    W,H=size; c.setPageSize(size)
    c.setFillColor(TEAL); c.setFont('Helvetica-Bold',9)
    c.drawString(42,H-36,'RESEARCH FORMULATION  |  PROPOSED METHOD')
    c.setFillColor(NAVY); c.setFont('Helvetica-Bold',18)
    c.drawString(42,H-65,title)
    c.setFont('Helvetica',10); c.setFillColor(INK); c.drawString(42,H-85,subtitle)
    c.setStrokeColor(RULE); c.line(42,H-96,W-42,H-96)
    c.setFont('Helvetica',8); c.setFillColor(INK)
    c.drawString(42,25,'DHLCT-C2-60 lift / DIHOOL actuator / Kinova Gen3 / CubeMars AK80 wheels')
    c.drawRightString(W-42,25,str(page))
    y=H-113

def para(text, use_style=style):
    global y
    replacements = {
      'omega_i^B': "<font name='ArialSymbols'>ω</font><sub>i</sub><super>B</super>",
      'p_C,i': 'p<sub>C,i</sub>', 'm_i': 'm<sub>i</sub>', 'R_i': 'R<sub>i</sub>', 'I_i': 'I<sub>i</sub>',
      'g_q':'g<sub>q</sub>', 'J_c':'J<sub>c</sub>', 'F_h':'F<sub>h</sub>', 'H_C':'H<sub>C</sub>',
      'M_O':'M<sub>O</sub>', 'F_z':'F<sub>z</sub>', 'e_3':'e<sub>3</sub>',
      'R_2(psi)':"R<sub>2</sub>(<font name='ArialSymbols'>ψ</font>)", 'o_S^W':'o<sub>S</sub><super>W</super>',
      'Q_Z':'Q<sub>Z</sub>', 'delta':"<font name='ArialSymbols'>δ</font>"
    }
    for key,value in replacements.items():text=text.replace(key,value)
    p=Paragraph(text,use_style); _,h=p.wrap(W-84,1000)
    if y-h<42: raise ValueError(f'Page overflow: {text[:40]} at {y}')
    p.drawOn(c,42,y-h); y-=h+5

def section(text):
    global y
    y-=4; c.setFont('Helvetica-Bold',11); c.setFillColor(TEAL); c.drawString(42,y,text); y-=18

def eq(name, maxwidth=None):
    global y
    im=ImageReader(str(EQ/(name+'.png'))); iw,ih=im.getSize()
    width=min(maxwidth or W-94,iw/3.2); height=width*ih/iw
    if y-height<42: raise ValueError('Equation overflow '+name)
    c.drawImage(im,(W-width)/2,y-height,width,height,mask='auto'); y-=height+3

def table(rows,widths):
    global y
    data=[[Paragraph(str(v),small) for v in row] for row in rows]
    t=Table(data,colWidths=widths,hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),LIGHT),('VALIGN',(0,0),(-1,-1),'TOP'),
      ('LINEBELOW',(0,0),(-1,0),.7,RULE),('LINEBELOW',(0,1),(-1,-1),.25,RULE),
      ('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),
      ('TOPPADDING',(0,0),(-1,-1),4),('BOTTOMPADDING',(0,0),(-1,-1),4)]))
    _,h=t.wrap(W-84,1000)
    if y-h<42: raise ValueError(f'Table overflow y={y}, height={h}')
    t.drawOn(c,42,y-h); y-=h+8

header('ZMP-based stability of the mobile manipulator','1. Mechanical model and derivation of the stability measure',1)
para('<b>Aim.</b> Define a physically interpretable tipping-stability objective for coordinated base, lift, and arm motion. The formulation uses Newton-Euler dynamics and Lie-group kinematics; it does not assume a particular control algorithm.')
section('1. System and mechanical inputs')
eq('state'); eq('kinematics'); eq('dynamics')
para('G is the full chassis pose; s contains lift height h, seven arm angles, four wheel angles, and passive roller angles. M is inertia, c is the velocity-dependent inertial term, g_q is gravity, d is resistance, B maps actuator effort, and J_c maps ground contact. The mechanical inputs are lift force F_h, seven arm torques and four wheel torques. Scissor-link motion is parameterized by h.')
section('2. Whole-system Newton-Euler balance')
eq('centermass'); eq('momentum')
para('For component i: m_i is mass, p_C,i is its world centre of mass, R_i is body-to-world rotation, I_i is inertia about its centre of mass, and omega_i^B is angular velocity in its own body frame. H_C is total angular momentum about the whole-system centre of mass. All terms in H_C are expressed in the world frame.',small)
eq('centroid')
para('F is the total ground force; M_O is its moment about a fixed floor origin; e_3 is the upward unit vector. These balances assume gravity and ground contact are the only external loads. An external tool wrench must be added when manipulating the environment.',small)
section('3. Zero-moment point on the floor')
eq('moment'); eq('zmpx'); eq('zmpy')
para('The floor is horizontal at z = 0. Positive F_z is required. The ZMP is the floor point where the horizontal components of the ground-wrench moment vanish. On this floor it coincides with the centre of pressure for a valid contact solution. Lift height and arm motion enter through centre-of-mass acceleration and angular-momentum rate.',small)
c.showPage()

header('Stability objective and constraints','2. A compact formulation suitable for an initial research proposal',2)
section('4. Express ZMP in the support frame')
para('Use a horizontal support frame S whose origin is the centre of the wheel footprint and whose axes follow chassis heading. Transform the world ZMP into that frame:')
eq('frame')
para('R_2(psi) is the planar rotation from S to the world horizontal frame; o_S^W is the footprint-centre position. The assumed four-wheel support region is [-a,a] x [-b,b]. The quantities a and b come from contact geometry, not the visual chassis dimensions.',small)
section('5. Proposed stability cost')
eq('stage'); eq('core'); eq('cost')
para('<b>Interpretation.</b> The cost is zero at the support centre and increases as the ZMP moves away. Q_Z normalizes each direction by the available footprint. No independent tuning weights or additional tilt penalties are required in this initial formulation.')
section('6. Feasibility and task requirements')
eq('bounds'); eq('opt')
para('The reserve delta accommodates estimation and model error. The cost encourages central support; the constraints preserve the reserve. Task constraints specify the desired motion and completion requirement; otherwise, remaining stationary can minimize stability cost.')
para('Additional constraints are the contact/friction law, positive supporting force, collision avoidance, and the actual joint, speed, torque, and lift-travel limits. If a wheel loses contact, replace the rectangle with the active contact polygon. ZMP containment alone is not a proof of recovery from arbitrary tilt or a guarantee against slip.',small)
section('Notation for the cost')
table([
 ['Symbol','Definition','Units'],
 ['x<sub>Z</sub><super>S</super>, y<sub>Z</sub><super>S</super>','ZMP coordinates in the support frame','m'],
 ['a, b','Support half-length and half-width','m'],
 ['Q<sub>Z</sub>','diag(1/a<super>2</super>, 1/b<super>2</super>)','1/m<super>2</super>'],
 ["<font name='ArialSymbols'>δ</font>",'Reserved distance from each support edge','m'],
 ['T; N; k','Motion duration; equal-interval sample count; sample index','s; -; -'],
 ],[106, W-84-106-88,88])
c.showPage()

header('Proposed hardware and feedback architecture','3. The robot image is the plant; controller choice remains open',3,landscape(A4))

def box(x,bottom,w,h,title,lines=(),fill=LIGHT):
    c.setFillColor(fill); c.setStrokeColor(TEAL if fill==LIGHT else NAVY); c.setLineWidth(1.2)
    c.roundRect(x,bottom,w,h,8,fill=1,stroke=1)
    c.setFillColor(NAVY); c.setFont('Helvetica-Bold',11)
    c.drawCentredString(x+w/2,bottom+h-22,title)
    c.setFont('Helvetica',9)
    for j,line in enumerate(lines): c.drawCentredString(x+w/2,bottom+h-43-j*14,line)

def arrow(points):
    c.setStrokeColor(NAVY); c.setFillColor(NAVY); c.setLineWidth(1.4)
    p=c.beginPath();p.moveTo(*points[0])
    for pt in points[1:]:p.lineTo(*pt)
    c.drawPath(p)
    x0,y0=points[-2];x,y1=points[-1]
    import math
    ang=math.atan2(y1-y0,x-x0); length=7;spread=.45
    p=c.beginPath();p.moveTo(x,y1)
    p.lineTo(x-length*math.cos(ang-spread),y1-length*math.sin(ang-spread))
    p.lineTo(x-length*math.cos(ang+spread),y1-length*math.sin(ang+spread));p.close()
    c.drawPath(p,fill=1,stroke=0)

box(35,290,96,80,'Task reference',['Base / tool goal','Lift configuration'])
box(163,270,158,120,'Command selection',['Task tracking','Minimize ZMP cost','Enforce support and','actuator constraints'])
box(356,263,160,134,'Inner controls',['Lift: position loop','Arm: position interface','Wheels: torque drivers','(AK80 mode-dependent)'])
box(555,235,215,225,'Coupled robot',[],white)
c.drawImage(str(ROBOT),582,264,162,163,preserveAspectRatio=True,anchor='c',mask='auto')
c.setFont('Helvetica',9);c.setFillColor(INK);c.drawCentredString(662.5,247,'Base + lift + arm + wheel contact')
box(359,110,225,95,'State / ZMP estimation',['Joint feedback + base-state sensing','Rigid-body model / contact estimate','ZMP relative to support polygon'])
arrow([(131,330),(163,330)])
arrow([(321,330),(356,330)])
arrow([(516,330),(555,330)])
arrow([(770,330),(804,330),(804,155),(584,155)])
arrow([(359,155),(242,155),(242,270)])
c.setFont('Helvetica',8);c.setFillColor(INK)
c.drawCentredString(339,408,'Command targets')
c.drawCentredString(535,408,'Effort')
c.drawCentredString(688,184,'State / model feedback')
c.drawCentredString(246,130,'Estimated state and ZMP')

# Compact hardware distinction under the diagram.
p=Paragraph("<b>Hardware command:</b> u<sub>cmd</sub> = (a<sub>ref</sub>, <font name='ArialSymbols'>θ</font><sub>ref</sub>, <font name='ArialSymbols'>τ</font><sub>W,ref</sub>): actuator-extension target, arm-position targets, and wheel-torque targets. The lift potentiometer closes its position loop; it does not measure force. Torque-controlled wheels assume a compatible AK80 driver mode. Inner controls produce the mechanical force/torque inputs.",small)
_,ph=p.wrap(W-84,1000);p.drawOn(c,42,91-ph)
c.save()

reader=PdfReader(str(PDF)); assert len(reader.pages)==3
doc=pdfium.PdfDocument(str(PDF))
for i in range(len(doc)):
    page=doc[i]; bmp=page.render(scale=1.6)
    bmp.to_pil().save(Path(__file__).parent/f'stability_page_{i+1}.png')
    if i==2:
        bmp.to_pil().save(OUT/'zmp_control_block_diagram.png')
    page.close()
doc.close()
print(PDF)
print('3 pages rendered for visual review')
