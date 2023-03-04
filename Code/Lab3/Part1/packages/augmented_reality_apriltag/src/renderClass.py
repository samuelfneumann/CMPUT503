# ---------------------------------------------------------------------------------
#
#
#
#                           DO NOT EDIT THE LINES BELOW
#
#
#
# ---------------------------------------------------------------------------------
import numpy as np
import cv2

class Renderer():

    def __init__(self, obj_path):
        """
        :param obj_path: is the path to the .obj model file to render
        """
        """Loads a Wavefront OBJ file. """
        self.vertices = []
        self.normals = []
        self.texcoords = []
        self.faces = []

        material = None
        for line in open(obj_path, "r"):
            if line.startswith('#'): continue
            values = line.split()
            if not values: continue
            if values[0] == 'v':
                v = list(map(float, values[1:4]))
                v = v[0], v[2], v[1]
                self.vertices.append(v)
            elif values[0] == 'vn':
                v = list(map(float, values[1:4]))
                v = v[0], v[2], v[1]
                self.normals.append(v)
            elif values[0] == 'vt':
                self.texcoords.append(list(map(float, values[1:3])))
            elif values[0] in ('usemtl', 'usemat'):
                material = values[1]
            elif values[0] == 'f':
                face = []
                texcoords = []
                norms = []
                for v in values[1:]:
                    w = v.split('/')
                    face.append(int(w[0]))
                    if len(w) >= 2 and len(w[1]) > 0:
                        texcoords.append(int(w[1]))
                    else:
                        texcoords.append(0)
                    if len(w) >= 3 and len(w[2]) > 0:
                        norms.append(int(w[2]))
                    else:
                        norms.append(0)
                self.faces.append((face, norms, texcoords, material))

    def __hex_to_rgb(self, hex_color):
        """
        Helper function to convert hex strings to RGB
        """
        hex_color = hex_color.lstrip('#')
        h_len = len(hex_color)
        return tuple(int(hex_color[i:i + h_len // 3], 16) for i in range(0, h_len, h_len // 3))

    def render(self, img, projection_matrix):
        """
        Render a loaded obj model into the current video frame.
        :param img: opencv image on which the model will be rendered.
        :param projection_matrix: numpy array 3x4 floating point matrix and it is supposed to be the projection
        matrix that represents the transformation from the AprilTag reference frame to the image frame.
        :return ima: opencv image with rendered model
        """


        for face in self.faces:
            face_vertices = face[0]
            points = np.array([self.vertices[vertex - 1] for vertex in face_vertices])
            points = np.array([[p[0], p[1], p[2]] for p in points])
            dst = cv2.perspectiveTransform(points.reshape(-1, 1, 3), projection_matrix)
            imgpts = np.int32(dst)
            color = self.__hex_to_rgb(face[-1])
            color = color[::-1]  # reverse
            cv2.fillConvexPoly(img, imgpts, color)

        return img