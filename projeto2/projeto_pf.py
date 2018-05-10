#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles, draw_random_sample
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math
import mpmath
import copy
from scipy.stats import norm
largura = 775 # largura do mapa
altura = 748  # altura do mapa

# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
particulas = []

num_particulas = 10000


# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=8, endpoint=False)

# Lista mais longa
movimentos_longos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0],
              [0,0,math.pi/12.0], [0, 0, math.pi/12.0], [0, 0, math.pi/12],[0,0,-math.pi/4],
              [-5, 0, 0],[-5,0,0], [-5,0,0], [-10,0,0],[-10,0,0], [-10,0,0],[-10,0,0],[-10,0,0],[-15,0,0],
              [0,0,-math.pi/4],[0, 10, 0], [0,10,0], [0, 10, 0], [0,10,0], [0,0,math.pi/8], [0,10,0], [0,10,0], 
              [0,10,0], [0,10,0], [0,10,0],[0,10,0],
              [0,0,-math.radians(90)],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0]]

# Lista curta
movimentos_curtos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0]]

movimentos_relativos = [[0, -math.pi/3],[10, 0],[10, 0], [10, 0], [10, 0],[15, 0],[15, 0],[15, 0],[0, -math.pi/2],[10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [0, -math.pi/2], 
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [0, -math.pi/2], 
                       [10,0], [0, -math.pi/4], [10,0], [10,0], [10,0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0]]



movimentos = movimentos_relativos



def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):
    """
        Cria uma lista de partículas distribuídas de forma uniforme entre minx, miny, maxx e maxy
    """
    grid = int(math.sqrt(n_particulas))
    particle_cloud = []
    map_area = altura*largura
    part = map_area/n_particulas
    part_side_x = largura/grid
    part_side_y = altura/grid
    part_center = (part_side_x/2,part_side_y/2)
    particles = []
    
    for i in range(0,grid):
        x = part_center[0]+(part_side_x*i)
        for k in range (0,grid):
            y = part_center[1] +(part_side_y*k)
            theta = np.random.uniform(0, 2*math.pi)
            p = Particle(x,y,theta,w=1.0)
            particles.append(p)
    
    return particles
    
def move_particulas(particulas, movimento):
    """
        Recebe um movimento na forma [deslocamento, theta]  e o aplica a todas as partículas
        Assumindo um desvio padrão para cada um dos valores
        Esta função não precisa devolver nada, e sim alterar as partículas recebidas.
        
        Sugestão: aplicar move_relative(movimento) a cada partícula
        
        Você não precisa mover o robô. O código fornecido pelos professores fará isso
        
    """
    
        

    mu_x,sigma_x = movimento[0],4
    mu_y,sigma_y = movimento[0],4
    mu_theta,sigma_theta = movimento[1],math.radians(3)

    for i in particulas:
        dx = np.random.normal(mu_x,sigma_x,1)
        dtheta = np.random.normal(mu_theta,sigma_theta,1)
        speed = (dx[0],dtheta[0])
        i.move_relative(speed)

    return particulas
    
def leituras_laser_evidencias(robot, particulas):
    """
        Realiza leituras simuladas do laser para o robo e as particulas
        Depois incorpora a evidência calculando
        P(H|D) para todas as particulas
        Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
        responde somente P(Hi|D), em que H é a hi
        
        Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
        
        Você vai precisar calcular para o robo
        
    """
    sigma = 10
    leitura_robo = inspercles.nb_lidar(robot, angles)
    alpha = 0
    prob_particulas = []
    for p in particulas:
        leituras = inspercles.nb_lidar(p,angles)
        prob = 0
        # p.w = p.w
        for laser in leituras.keys():
            # prob+=norm.pdf(leituras[laser],loc=leitura_robo[laser],scale=sigma)
            prob += math.exp((-(leituras[laser]-leitura_robo[laser])**2)/(2*math.pow(sigma,2)))
        p.w *= prob
        alpha += prob

    for robot in particulas:
        robot.w /= alpha
   # Voce vai precisar calcular a leitura para cada particula usando inspercles.nb_lidar e depois atualizar as probabilidades


    
    
def reamostrar(particulas, n_particulas = num_particulas):
    """
        Reamostra as partículas devolvendo novas particulas sorteadas
        de acordo com a probabilidade e deslocadas de acordo com uma variação normal    
        
        O notebook como_sortear tem dicas que podem ser úteis
        
        Depois de reamostradas todas as partículas precisam novamente ser deixadas com probabilidade igual
        
        Use 1/n ou 1, não importa desde que seja a mesma
    """
    prob_all = [ p.w for p in particulas]

    particulas = draw_random_sample(particulas,prob_all,num_particulas)

    particulas = [desv(p,5,math.radians(5),True) for p in particulas]


    return particulas

def desv (p,linear_sigma,angle_sigma,reset_prob=False):
    if reset_prob:
        p.w=1

    d = np.random.normal(0,linear_sigma,2)
    
    dx=d[0]
    dy=d[1]
    dtheta = np.random.normal(0,angle_sigma,1)
    
    p.move_angular(dtheta[0])
    p.x += dx
    p.y += dy

    return p
    







