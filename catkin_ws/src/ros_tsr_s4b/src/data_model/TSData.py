class Region:
    def __init__(self,x1,y1,x2,y2,label=None,score=None):
        #assert x1<x2, "x1 must be the part of the top left window corner"
        #assert y1<y2, "y1 must be the part of the top left window corner"
        self.label = label
        self.score = score
        self.x1=x1
        self.y1=y1
        self.x2=x2
        self.y2=y2
        self.w=x2-x1
        self.h=y2-y1
        self.p1 = (x1,y1)
        self.p2 = (x2,y2)