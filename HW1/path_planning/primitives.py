from dataclasses import dataclass


class PixelCoordinates:
    def __init__(self, x:int, y:int):
        self._x = round(x)
        self._y = round(y)

    @property
    def x(self):
        return self._x
    
    @property
    def y(self):
        return self._y
    
    def __sub__(self, other):
        if not isinstance(other, PixelCoordinates):
            raise TypeError()
        return PixelVector(
            x=self.x - other.x, 
            y=self.y - other.y
        )

    def to_tuple(self):
        return (self.x, self.y)

    def __eq__(self, other):
        if isinstance(other, PixelCoordinates):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        return hash((self.x, self.y))
    
    def __repr__(self):
        return f"PixelCoordinates({self.x}, {self.y})"
    
class PixelVector(PixelCoordinates):
    def __add__(self, other):
        if not isinstance(other, PixelVector):
            raise TypeError()
        return PixelVector(
            x=self.x + other.x, 
            y=self.y + other.y
        )
    
    def __sub__(self, other):
        if not isinstance(other, PixelVector):
            raise TypeError()
        return PixelVector(
            x=self.x - other.x, 
            y=self.y - other.y
        )
    
    def __repr__(self):
        return f"PixelVector({self.x}, {self.y})"
    
@dataclass
class PathNode:
    coordinates:PixelCoordinates
    parent:"PathNode"=None
    cost:float=0

    def __eq__(self, other):
        if isinstance(other, PathNode):
            return self.coordinates == other.coordinates
        return False

    def __hash__(self):
        return hash(self.coordinates)