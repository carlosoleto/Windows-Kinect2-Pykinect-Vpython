from visual import *
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes

#import matplotlib.pyplot as plt
import numpy as np
import csv

import sys

REF_COXA = 0
REF_PERNA = 1
REF_PE = 2

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread
	
"""
Documentacao
Eixos do visual python sao os mesmo da API do Kinect.

Eixo X - Horizontal - positivo para direita do usuario
Eixo Y - Vertical - positivo para cima do usuario
Eixo Z - produndidade - positivo para longe do Kinect

O sensor kinect pode ter sua orientacao em relacao a vertical,
em torno do eixo X, alterada facilmente.

Por isso, para que a simulacao seja vista com mais facilidade
pelo os responsaveis pelo teste, eh necessario rotacionar o
eixo de coordenadas do mundo criado em torno do Kinect.

O calculo da inclinacao em relacao ao piso sao feitos com
uso do objeto floor_clip_plane. Retorna vetor normal ao piso

Para compensar esse valor na trajetoria do movimento referencia
o angulo foi calculado 

self.angRotX = np.arctan(piso.z/piso.y)

E adicionado no angulo referencia lido do arquivo .txt

Componentes relacionados a rotacao do eixo Z ainda nao foram
tratadas, contudo a rotina de rotacao do eixo pode fazer isso
em um unico passo
"""
	
	
"""
JointType_SpineBase = 0
JointType_SpineMid = 1
JointType_Neck = 2
JointType_Head = 3
JointType_ShoulderLeft = 4
JointType_ElbowLeft = 5
JointType_WristLeft = 6
JointType_HandLeft = 7
JointType_ShoulderRight = 8
JointType_ElbowRight = 9
JointType_WristRight = 10
JointType_HandRight = 11
JointType_HipLeft = 12
JointType_KneeLeft = 13
JointType_AnkleLeft = 14
JointType_FootLeft = 15
JointType_HipRight = 16
JointType_KneeRight = 17
JointType_AnkleRight = 18
JointType_FootRight = 19
JointType_SpineShoulder = 20
JointType_HandTipLeft = 21
JointType_ThumbLeft = 22
JointType_HandTipRight = 23
JointType_ThumbRight = 24
JointType_Count = 25
"""

class Skeleton:
    """Kinect skeleton represented as a VPython frame.
    """
	
    def __init__(self, f):
        """Create a skeleton in the given VPython frame f.
        """
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
        self._bodies = None
        self._currentBody = 0
        self.refCounter = 0
        self.frame = f
        self.bones = [cylinder(frame=f, radius=0.02, color=color.yellow)
                      for bone in _bone_ids2]
        c1 = cylinder(frame=f, radius=0.03, color=color.red)
        #c2 = cylinder(frame=f, radius=0.03, color=color.blue)
        #c3 = cylinder(frame=f, radius=0.03, color=color.red)
        #self.ref = [c1,c2,c3]
        self.ref = [c1]
        
        cPisoY = cylinder(frame=f, radius=0.01, color=color.blue)
        #cPisoZ = cylinder(frame=f, radius=0.01, color=color.green)
        #self.vetoresPiso = [cPisoY,cPisoZ]
        self.vetoresPiso = [cPisoY]
        
        self.piso = None
        
        self.angRotX = 0
        self.angRotZ = 0
        
        #scene.forward = (-1,0,0)

    def find_Current_Body(self):
        #pega a lista e procura o menor valor para posixao z
        tempDist = 1000;
        current = 10;
        for i in range(0, self._kinect.max_body_count):
            body = self._bodies.bodies[i]
            if body.is_tracked:
                joints = body.joints
                if joints[PyKinectV2.JointType_Head].Position.z < tempDist:
                    #guardar valor
                    current = i
                    tempDist = joints[PyKinectV2.JointType_Head].Position.z

        return int(current)

    def drawReference(self,joints,piso,refHip):

		self.vetoresPiso[0].pos = (0,0,0)
		self.vetoresPiso[0].axis = 	(piso.x,piso.y,piso.z)
		
		self.angRotX = np.arctan(piso.z/piso.y)
		self.angRotZ = np.arctan(piso.x/piso.y)
		
		#print(str(np.degrees(self.angRotX)) + " -- " + str(np.degrees(self.angRotZ)))
		
		#atualizar contador
		self.refCounter = self.refCounter + 1
		if self.refCounter == len(refHip):
			self.refCounter = 0
			
		#coxa
		pQuadril = joints[PyKinectV2.JointType_HipRight].Position
		pJoelho = joints[PyKinectV2.JointType_KneeRight].Position
		#pJoelho.z = pJoelho.z -0.2
		
		#pegando tamanho da coxa
		tamanhoCoxa = np.sqrt((pJoelho.x - pQuadril.x)**2+
							(pJoelho.y - pQuadril.y)**2 + 
							(pJoelho.z - pQuadril.z)**2)
		
		#deltaZ = (-1)*tamanhoCoxa*np.sin(np.radians(refHip[self.refCounter]) + self.angRotX)
		#deltaY = (-1)*tamanhoCoxa*np.cos(np.radians(refHip[self.refCounter]) + self.angRotX)
		
		deltaZ = (-1)*tamanhoCoxa*np.sin(np.radians(refHip[self.refCounter]))
		deltaY = (-1)*tamanhoCoxa*np.cos(np.radians(refHip[self.refCounter]))
		
		#print(str(deltaZ) + " " + str(deltaY))
		
		pJoelho.z = pQuadril.z + deltaZ
		pJoelho.y = pQuadril.y + deltaY
		pJoelho.x = pQuadril.x + 0
		
		##########################################################################
		pTornozelo = joints[PyKinectV2.JointType_AnkleRight].Position
		#pTornozelo.z = pTornozelo.z -0.2
		
		#pegando tamanho da perna
		
		
		pPe = joints[PyKinectV2.JointType_FootRight].Position
		pPe.z = pPe.z -0.2
		
		#atualizar vetores com rotacoes no eixo
		
		vQuadril = rotate(vector(pQuadril.x,pQuadril.y,pQuadril.z),(-1)*self.angRotX,(1,0,0))
		vQuadril = rotate(vQuadril,self.angRotZ,(0,0,1))
		
		vJoelho = rotate(vector(pJoelho.x,pJoelho.y,pJoelho.z),(-1)*self.angRotX,(1,0,0))
		vJoelho = rotate(vJoelho,self.angRotZ,(0,0,1))
		
		self.ref[REF_COXA].pos = (vQuadril.x,vQuadril.y,vQuadril.z)
		self.ref[REF_COXA].axis = 	(vJoelho.x - vQuadril.x,
									 vJoelho.y - vQuadril.y,
									 vJoelho.z - vQuadril.z)
									 
		#perna
		#self.ref[REF_PERNA].pos = (pJoelho.x,pJoelho.y,pJoelho.z)
		#self.ref[REF_PERNA].axis = 	(pTornozelo.x - pJoelho.x,
		#							 pTornozelo.y - pJoelho.y,
		#							 pTornozelo.z - pJoelho.z)
		#pe
		#self.ref[REF_PE].pos = (pTornozelo.x,pTornozelo.y,pTornozelo.z)
		#self.ref[REF_PE].axis = 	(pPe.x - pTornozelo.x,
		#							 pPe.y - pTornozelo.y,
		#							 pPe.z - pTornozelo.z)
									 
		#print(str(pQuadril.x) +' ' + str(pQuadril.y) +' '+ str(pQuadril.z))
		
									 
    def update(self,refHip):
        """Update the skeleton joint positions in the depth sensor frame.

        Return true iff the most recent sensor frame contained a tracked
        skeleton.
        """
        updated = False
        if self._kinect.has_new_body_frame(): 
            self._bodies = self._kinect.get_last_body_frame()
        
        # atualizar posicao dos ossos.
        if self._bodies is not None:
            # encontrar pessoa  mais proxima
            pessoaProxima = self.find_Current_Body()
            if pessoaProxima < 10:
                body = self._bodies.bodies[pessoaProxima]
                self.piso = self._bodies.floor_clip_plane
                
                joints = body.joints

                """
                for i in range(0,len(_bone_ids2)):
                    # pegar o cilindro correspondente
                    bone = self.bones[i]
                    print(bone)
                    print(bone.pos)
                    print(bone.axis)
                    #pegando indices
                    p1 = _bone_ids2[i][0]
                    p2 = _bone_ids2[i][1]
                    #pegando posicoes
                    pos1 = joints[p1].Position
                    pos2 = joints[p2].Position

                    print(pos1.x)
                    print(pos2.y)

                    bone.pos = pos1
                    bone.axis = pos2 - pos1
                """
                #joints[PyKinectV2.JointType_HandRight].Position.x
                #atualizar cada um dos ossos 
                #self.bons -- lista de cilindros
                #_bone_ids2 -- lista de pares de coordenadas
                
                for bone, bone_id in zip(self.bones, _bone_ids2):
                    p1, p2 = [joints[id].Position for id in bone_id]
                    #p2 - ponto final
                    #p1 - ponto inicial
                    
                    #print(p1)
                    #print(p1.x,p1.y)
                    #print(bone.pos)
                    
                    #tentativa de rotacao em X e em Z
                    v1 = rotate(vector(p1.x,p1.y,p1.z),(-1)*self.angRotX,(1,0,0)) #rotacao em X
                    v1 = rotate(v1,self.angRotZ,(0,0,1)) #rotacao em Z
                    v2 = rotate(vector(p2.x,p2.y,p2.z),(-1)*self.angRotX,(1,0,0)) #rotacao em X
                    v2 = rotate(v2,self.angRotZ,(0,0,1)) #rotacao em Z
					
                    bone.pos = (v1.x,v1.y,v1.z)
                    bone.axis = (v2.x-v1.x,v2.y-v1.y,v2.z-v1.z)
                    
                    #bone.pos = (p1.x,p1.y,p1.z)
                    #bone.axis = (p2.x-p1.x,p2.y-p1.y,p2.z-p1.z)

                #atualizar rederencia
                # skeleton.drawReference(joints,self.piso,refHip)
                
                #mudando camera
                pMeioColuna = joints[PyKinectV2.JointType_SpineMid].Position
				
                scene.center = (pMeioColuna.x,pMeioColuna.y,pMeioColuna.z)
                scene.forward = (-1,0,0)
                #teste informacao piso
                #print("Teste (" + str(self.piso.x) + " " + str(self.piso.y) + " " + str(self.piso.z)

                updated = True
        return updated

def draw_sensor(f):
    """Draw 3D model of the Kinect sensor.

    Draw the sensor in the given (and returned) VPython frame f, with
    the depth sensor frame aligned with f.
    """
    box(frame=f, pos=(0, 0, 0), length=0.2794, height=0.0381, width=0.0635,
        color=color.blue)
    cylinder(frame=f, pos=(0, -0.05715, 0), axis=(0, 0.0127, 0), radius=0.0381,
             color=color.blue)
    cone(frame=f, pos=(0, -0.04445, 0), axis=(0, 0.01905, 0), radius=0.0381,
         color=color.blue)
    cylinder(frame=f, pos=(0, -0.05715, 0), axis=(0, 0.0381, 0), radius=0.0127,
             color=color.blue)
    cylinder(frame=f, pos=(-0.0635, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    cylinder(frame=f, pos=(-0.0127, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    cylinder(frame=f, pos=(0.0127, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    text(frame=f, text='KINECT', pos=(0.06985, -0.00635, 0.03175),
         align='center', height=0.0127, depth=0.003)
    #colocando eixos
    #cylinder(frame=f, pos=(0,0,0), axis=(1,0,0), radius=0.001,color=color.red)
    #cylinder(frame=f, pos=(0,0,0), axis=(0,1,0), radius=0.001,color=color.green)
    #cylinder(frame=f, pos=(0,0,0), axis=(0,0,1), radius=0.001,color=color.blue)
    
    return f

# A bone is a cylinder connecting two joints, each specified by an id.
_bone_ids2 = [
    # Torso
    [PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck],
    [PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder],
    [PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid],
    [PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase],
    [PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight],
    [PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft],
    [PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight],
    [PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft ],
    # Braco direito
    [PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight],
    [PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight],
    [PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight],
    # Braco esquerdo
    [PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft],
    [PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft],
    [PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft],
    # Perna direita
    [PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight],
    [PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight],
    [PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight],
    # Perna esquerda
    [PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft],
    [PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft],
    [PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft]]

_bone_ids = [
    [0, 1], [1, 2], [2, 3], [7, 6], [6, 5], [5, 4], [4, 2],
             [2, 8], [8, 9], [9, 10], [10, 11], [15, 14], [14, 13], [13, 12],
             [12, 0], [0, 16], [16, 17], [17, 18], [18, 19]]

if __name__ == '__main__':
	draw_sensor(frame())
	skeleton = Skeleton(frame(visible=False))
	hipAngles = np.loadtxt('HipFlexExt.txt')
	hipAngles = hipAngles[:,0]
	
	kneeAngles = np.loadtxt('KneeFlexExt.txt')
	kneeAngles = kneeAngles[:,0]
	
	#print(hipAngles)
	#print(np.pi)
	
	while True:
	    rate(30)
	    skeleton.frame.visible = skeleton.update(hipAngles)

