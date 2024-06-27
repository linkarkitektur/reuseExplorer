from specklepy.objects.geometry import Plane, Point, Vector

WORLDXY = Plane(
    origin=Point(x=0.0,y=0.0, z=0.0),
    normal=Vector(x=0.0,y=0.0, z=1.0),
    xdir=Vector(x=1.0,y=0.0, z=0.0),
    ydir=Vector(x=0.0,y=1.0, z=0.0))