
from os import listdir
from cv2 import imread,resize,IMREAD_UNCHANGED
class TSModelRepository():
    def __init__(self,symbol_dir_path):

        self._sample_paths = [(symbol_dir_path + "/" + name, name.replace(".png", "")) for name in listdir(symbol_dir_path)];
        self._symbols = [(imread(sample_path[0], IMREAD_UNCHANGED), sample_path[1]) for sample_path in self._sample_paths]
    def getSymbolByName(self,name):
        for symbol in self._symbols:
            if symbol[1]==name:
                return symbol[0]
        return None
    def getSymbolPathByName(self,name):
        '''
        :param name: Name of the Traffic Sign
        :return:
        '''
        for symbol_path in self._sample_paths:
            if symbol_path[1]==name:
                return symbol_path[0]
        return None
    def transparentOverlay(self,src, overlay, pos=(0, 0), scale=1, alpha_v=1.0):
        """
        :param src: Input Color Background Image
        :param overlay: transparent Image (BGRA)
        :param pos:  position where the image to be blit.
        :param scale : scale factor of transparent image.
        :return: Resultant Image
        """
        overlay = resize(overlay, (0, 0), fx=scale, fy=scale)
        h, w, _ = overlay.shape  # Size of foreground
        rows, cols, _ = src.shape  # Size of background Image
        y, x = pos[0], pos[1]  # Position of foreground/overlay image

        # loop over all pixels and apply the blending equation
        for i in range(h):
            for j in range(w):
                if x + i >= rows or y + j >= cols:
                    continue
                alpha = float(overlay[i][j][3] / 255) * alpha_v  # read the alpha channel
                src[x + i][y + j] = alpha * overlay[i][j][:3] + (1 - alpha) * src[x + i][y + j]
        return src
class TrafficSign():
    def __init__(self):
        self._region = None
        self._symbol = None
