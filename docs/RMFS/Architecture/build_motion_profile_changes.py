"""Rebuild the vector appendix illustration; run with the bundled reportlab Python."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor, Color

OUTPUT = Path(__file__).parent/'assets'/'motion_profile_changes.pdf'
W,H = 1500,760
c=canvas.Canvas(str(OUTPUT),pagesize=(W,H))
ink=HexColor('#243746');blue=HexColor('#0078aa');purple=HexColor('#773da6');grey=HexColor('#8b96a1')
def text(x,y,s,size=13,bold=False,color=ink):
    c.setFillColor(color);c.setFont('Helvetica-Bold' if bold else 'Helvetica',size);c.drawCentredString(x,y,s)
def arrow(a,b,label):
    c.setStrokeColor(purple);c.setLineWidth(1.8);c.line(*a,*b)
    x,y=b;c.line(x,y,x-7,y+4);c.line(x,y,x-7,y-4)
    text((a[0]+b[0])/2,a[1]+52,label,10,True,purple)
text(W/2,722,'Motion profile changes commanded after actual sweep release',25,True)
text(W/2,689,'No MRobot request and no lookahead: RMFS reserves the next safe endpoint, then commands extension.',15)
cases=[('Continue accelerating','Replace planned braking with continued acceleration',3,
        [(0,0),(3,1.2)],[(3,1.2),(6,0),(16,0)],[(3,1.2),(5,2),(9,2),(14,0),(16,0)]),
       ('Extend cruising','Keep cruising instead of starting planned braking',7,
        [(0,0),(4,2),(7,2)],[(7,2),(11,0),(16,0)],[(7,2),(10,2),(14,0),(16,0)]),
       ('Resume acceleration','Cancel ongoing braking before reaching rest',8,
        [(0,0),(4,2),(6,2),(8,1)],[(8,1),(10,0),(16,0)],[(8,1),(10,2),(12,2),(16,0)])]
for k,(title,desc,ts,history,old,new) in enumerate(cases):
    x=20+k*500
    text(x+230,638,f'({chr(97+k)}) {title}',18,True)
    text(x+230,611,desc,12)
    boxes=[(x+5,'RMFSController','Actual release / FIFO'),(x+185,'MRobot','Replace motion profile'),(x+365,'PSensor','Replace pending schedule')]
    for bx,title2,sub in boxes:
        c.setFillColor(HexColor('#f4f7fa'));c.setStrokeColor(HexColor('#aebdca'))
        c.roundRect(bx,487,100,76,7,fill=1,stroke=1)
        text(bx+50,536,title2,11,True);text(bx+50,510,sub,7.5)
    arrow((x+105,525),(x+185,525),'?extend_motion')
    arrow((x+285,525),(x+365,525),'!motion_plan')
    ox,oy=x+48,149; sx,sy=25,116
    def point(p):return ox+p[0]*sx,oy+p[1]*sy
    c.setLineWidth(1);c.setStrokeColor(ink);c.line(ox,oy,ox+410,oy);c.line(ox,oy,ox,oy+270)
    for v in (0,1,2):
        text(ox-18,oy+v*sy-4,str(v),11)
        if v:
            c.setStrokeColor(HexColor('#dde5eb'));c.line(ox,oy+v*sy,ox+400,oy+v*sy)
    for t in (0,4,8,12,16):text(ox+t*sx,oy-22,str(t),11)
    text(ox+5,oy+287,'v (m/s)',12);text(ox+392,oy-45,'t (s)',12)
    for path,color,dash in ((old,grey,True),(history,ink,False),(new,blue,False)):
        c.setStrokeColor(color);c.setLineWidth(3);c.setDash(6,4) if dash else c.setDash()
        p=c.beginPath();p.moveTo(*point(path[0]))
        for pos in path[1:]:p.lineTo(*point(pos))
        c.drawPath(p)
    c.setDash(3,4);c.setStrokeColor(purple);px,py=point(history[-1]);c.line(px,oy,px,oy+260);c.setDash()
    c.setFillColor(purple);c.circle(px,py,4,fill=1,stroke=0)
    text(x+250,450,f'Extension command received at t_c = {ts} s',12,True,purple)
text(W/2,70,'Dark: executed history     Dashed gray: superseded plan     Blue: revised motion     Purple: command reception',13)
text(W/2,42,'Position and speed remain continuous. PSensor reschedules only future clearances at the same simulation time.',13)
c.showPage();c.save()
