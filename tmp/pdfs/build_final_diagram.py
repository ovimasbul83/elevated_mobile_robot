from pathlib import Path
import math
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader
from reportlab.lib.colors import black,white
import pypdfium2 as pdfium

root=Path(__file__).resolve().parents[2]
out=root/'output/pdf'; eqdir=Path(__file__).parent/'final_diagram_equations'
pdfpath=out/'zmp_final_block_diagram.pdf'
c=canvas.Canvas(str(pdfpath),pagesize=(1120,760))
c.setTitle('Block diagram for the final ZMP stability formulation')
def box(x,y,w,h,title):
 c.setFillColor(white);c.setStrokeColor(black);c.setLineWidth(1.5)
 c.rect(x,y,w,h,fill=1,stroke=1)
 c.setFillColor(black);c.setFont('Helvetica-Bold',16)
 c.drawCentredString(x+w/2,y+h-28,title)
def text(x,y,s,size=13):
 c.setFillColor(black);c.setFont('Helvetica',size);c.drawCentredString(x,y,s)
def eq(name,x,y,maxwidth=255,maxheight=35):
 im=ImageReader(str(eqdir/(name+'.png')));w,h=im.getSize()
 scale=min(1/3.2,maxwidth/w,maxheight/h)
 c.drawImage(im,x-w*scale/2,y-h*scale/2,w*scale,h*scale,mask='auto')
def arrow(pts):
 c.setStrokeColor(black);c.setFillColor(black);c.setLineWidth(1.5)
 p=c.beginPath();p.moveTo(*pts[0])
 for pt in pts[1:]:p.lineTo(*pt)
 c.drawPath(p)
 x0,y0=pts[-2];x,y=pts[-1];a=math.atan2(y-y0,x-x0)
 p=c.beginPath();p.moveTo(x,y)
 p.lineTo(x-9*math.cos(a-.4),y-9*math.sin(a-.4))
 p.lineTo(x-9*math.cos(a+.4),y-9*math.sin(a+.4));p.close()
 c.drawPath(p,fill=1,stroke=0)
text(560,723,'ZMP-Based Stability Cost: Block Diagram',25)
eq('input',560,680,maxwidth=800,maxheight=38)
text(560,643,'Lift extension target (m) | Arm joint torque (Nm) | Wheel torque commands (Nm)',13)
box(45,410,170,130,'Desired task')
text(130,475,'Base / arm motion');text(130,450,'Lift position')
box(300,390,280,180,'Controller / optimizer')
eq('control',440,515)
eq('marginx',440,477,230);eq('marginy',440,445,230)
text(440,414,'Task and actuator constraints')
box(755,350,300,250,'Robot system')
c.drawImage(str(out/'robot_reference.png'),815,388,180,174,preserveAspectRatio=True,anchor='c')
text(905,367,'Base + scissor lift + Kinova arm',12)
box(755,65,300,180,'Ground-wrench calculation')
eq('force',905,179);eq('moment',905,136)
text(905,94,'State / model-based evaluation',12)
box(395,65,300,180,'ZMP calculation')
eq('zmpx',545,173);eq('zmpy',545,131)
text(545,94,'Horizontal ground surface',12)
box(45,65,290,180,'Stability cost')
eq('stage',190,173);eq('cost',190,127,maxheight=50)
text(190,92,'Minimize displacement from centre',12)
arrow([(215,475),(300,475)])
arrow([(580,475),(755,475)]);eq('u',668,493)
arrow([(905,350),(905,245)]);text(990,295,'Robot state',12)
arrow([(755,155),(695,155)]);eq('wrench',725,178,55)
arrow([(395,155),(335,155)]);eq('xy',365,180,55)
arrow([(190,245),(190,320),(440,320),(440,390)]);eq('J',316,338)
text(190,44,'ZMP coordinates are relative to the support centre;',11)
text(190,27,'a and b are support half-dimensions.',11)
c.save()
doc=pdfium.PdfDocument(str(pdfpath));page=doc[0]
page.render(scale=1.8).to_pil().save(out/'zmp_final_block_diagram.png')
page.close();doc.close()
print(pdfpath)
