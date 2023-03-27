#!../.venv/bin/python3.11

import numpy as np
import time

class Component:
    ID = 1

    def __init__(self, x, y, w, h, a, g):
        # ===== Constants =========================================
        self.LEN_MAX_HISTORY = 10

        # Initializing this component
        self.x = x
        self.y = y
        self.width = w
        self.height = h
        self.area = a
        self.geometry = g

        self.lastUpdate = time.time()
        self.matrix = self.generate_matrix(1 , 1, 1)

        self.ID = None
        self.color = (0, 0, 0)
        self.similarity = 0

        self.history = []

        self.frequency = 0

    def generate_matrix(self, wCoordinates, wBounds, wArea):
        # Generating a 2D matrix of core coordinates, bounds and area.
        matrix = np.array([
            self.x * wCoordinates,
            self.y * wCoordinates,
            self.width * wBounds,
            self.height * wBounds,
            (self.area / 100) * wArea
        ])

        return matrix

    def update_component(self, ID, similarity, history=[]):
        self.lastUpdate = time.time()
        self.similarity = similarity

        self.ID = ID
        self.history = history

    def check_similarity_single(self, input_class, wCoordinates, wBounds, wArea) -> int:
        self.matrix = self.generate_matrix(wCoordinates, wBounds, wArea)
        inputMatrix = input_class.generate_matrix(wCoordinates, wBounds, wArea)
        
        bufferArray = np.abs(inputMatrix - self.matrix)
        mean = np.fix(np.mean(bufferArray))

        return mean

    def check_similarity_array(self, input_array, wCoordinates, wBounds, wArea):
        similarityList = []
        
        # We want to figure out which component in an array is the most similar to this class!
        for i, component in enumerate(input_array):
            mean = self.check_similarity_single(component, wCoordinates, wBounds, wArea)
            similarityList.append(np.array([i, mean]))

        # Converting the list to a numpy array and sorting it | lower values represent higher similarity | return the most similar value
        similarityArray = np.array(similarityList)

        try:
            similarityArray = similarityArray[np.argsort(similarityArray[:, 1])]
        except IndexError:
            pass   

        if len(similarityArray) > 0:
            return similarityArray[0, 0], similarityArray[0, 1]
        else:
            return None, None

    def get_real_coordinates(self):
        pass

    def generate_id(self):
        # Generate a unique identity
        self.ID = Component.ID
        Component.ID += 1
