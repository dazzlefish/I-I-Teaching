from ipycanvas import Canvas
import math

class Turtle:

    def __init__(self, canvas):
        self.pos = (0,0)
        self.dir = 0
        self.down = True
        self.ctx = canvas
        self.ctx.stroke_style = "blue"
        self.ctx.clear()
        self.ctx.translate(canvas.size[0]/2, canvas.size[1]/2)

    def forward(self, n):
        dx = round (n*math.cos(self.dir*math.pi/180))  ## from degrees to radians: *2pi/360
        dy = round (n*math.sin(self.dir*math.pi/180))
        x, y = self.pos
        newpos = (x+dx, y+dy)
        if self.down:
            self.ctx.begin_path()
            self.ctx.move_to(*self.pos)
            self.ctx.line_to(*newpos)
            self.ctx.stroke()
        self.pos = newpos
    
    def back(self, n):
        self.forward(-n)
    
    def right(self, d):
        self.dir = (self.dir + d) % 360
        
    def left(self, d):
        self.dir = (self.dir - d) % 360
        
    def to_position(self, x, y):
        self.pos = (x, y)
        
    def goto(self, x, y):
        self.pos = (x, y)
        
    def pen_up(self):
        self.down = False
        
    def pen_down(self):
        self.down = True
        
    def set_color(self, c):
        self.color = c
        
    def init(self):
        self.ctx.reset_transform()
        self.ctx.clear()
        self.ctx.translate(self.ctx.size[0]/2, self.ctx.size[1]/2)
        self.pos = (0, 0)
        self.down = True
        self.dir = 0
