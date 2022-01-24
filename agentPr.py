#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import de las librerias
from errno import ERANGE
import random
import time
import os
import numpy as np
from ns3gym import ns3env
import matplotlib.pyplot as plt

# Inicialización de las variables de entorno
port = 5555
simTime = 100
startSim = True
stepTime = 0.6
seed = 0
simArgs = {"--distance": 500}
debug = False

env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)

# Inicialización de la Q-tabla con los valores obtenidos del entorno
Q = np.zeros((2000, 2000))

# Parametros 
# La variable alpha determina el ratio de aprendizaje
alpha = 0.75                  
discount_factor = 0.95               
epsilon = 1                  
max_epsilon = 1
min_epsilon = 0.01         
decay = 0.01         

train_episodes = 30    
test_episodes = 100          
max_steps = 100 

# Episodios (Entrenamiento del agente)

# Lista para los valores de recompensa
training_rewards = []  

for episode in range(train_episodes):

    # Se restablece el entorno por cada episodio
    state = env.reset()    

    # Inicialización de las recompensas
    total_training_rewards = 0
    
    for step in range(100):
        
        # Se realiza una verificación y de este resultado se selecciona la acción a realizar
        action = env.action_space.sample()
            
        # A partir de la acción se procede a realizar el siguiente paso en el entorno obteniendo el nuevo estado y la recompensa
        new_state, reward, done, info = env.step(action)
        
        # Se procede a actualizar la Q-tabla utilizando la ecuación de Bellman
        Q[state, action] = Q[state, action] + alpha * (reward + discount_factor * np.max(Q[new_state, :]) - Q[state, action]) 
        
        # Se actualizan las variables con los resultados obtenidos
        total_training_rewards += reward      
        state = new_state         
        
        # Se finaliza el episodio
        if done == True:
            print ("Recompensa total del episodio {}: {}".format(episode, total_training_rewards))
            break
    
    # Se agrega la recompensa total y los valores de épsilon
    training_rewards.append(total_training_rewards)

    
print ("Puntuación de entrenamiento a lo largo del tiempo: " + str(sum(training_rewards)/train_episodes))

# Se cierra el entorno
env.close()

# Se imprimen los resultados obtenidos
# Se visualizan los resultados y la recompensa total en todos los episodios
x = range(train_episodes)
plt.plot(x, training_rewards)
plt.xlabel('Episodio')
plt.ylabel('Recompensa total del entrenamiento')
plt.title('Recompensas totales en todos los episodios de entrenamiento') 
plt.show()
