from lib.segmentor import Segmentation
from lib.converter import Converter

from icecream import ic

if __name__ == "__main__":
    segment = Segmentation('src\data\DFloor.ifc')
    segment.calc_wall()