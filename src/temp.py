# for component in components:
#             componentMatrix = np.array([
#                 component['index'],
#                 component['x'],
#                 component['y'],
#                 component['width'],
#                 component['height'],
#                 component['area']
#                 ])
            
#             stepArray.append(componentMatrix)

#             if len(self.history) > 0:
#                 filterArray = []

#                 for historicalMatrix in self.history[-1]:
#                     # Subtract the historical matrix from the current one; The smaller the value, the more similar they are!
#                     # NOTE: Implement WEIGHT for the individual components!!!
#                     bufferArray = np.abs(componentMatrix - historicalMatrix)

#                     mean = np.fix(np.mean(bufferArray))
#                     appendArray = np.array([historicalMatrix[0], mean])

#                     filterArray.append(appendArray)

#                 # Sort the filterArray value, while keeping the index. In this case the sortedArray represents how similar each historicalMatrix is to the current one!
#                 filterArray = np.array(filterArray)
#                 sortedIndices = np.argsort(filterArray[:, 1])
#                 sortedArray = filterArray[sortedIndices]

#                 # if sortedArray[0, 0] <= self.MAX_MATRIX_DIFFERENCE:
#                 print('Difference: %s' %sortedArray[0, 0])
#                 componentMatrix[0] = sortedArray[0, 0]

#                 # NOTE: Reduce duplicate 'closest arrays' by checking which one is actually the closest! Also the one which is not close will be given an index outside the current bounds.
    
#                 subStepArray.append(componentMatrix)

#         if len(subStepArray) > 0:
#             sortedSubStepArray = np.zeros((len(subStepArray), 6))

#             subStepArray = np.array(subStepArray)
#             sortedIndices = np.argsort(subStepArray[0])

#             print('Indices: %s' %len(sortedIndices))
#             print('SSArr: %s' %len(subStepArray))

#             for i in sortedIndices:
#                 if i < len(subStepArray):
#                     sortedSubStepArray[i] = subStepArray[i]

#             print(subStepArray)

#         Avoid using too much memory, popping history.
#         if len(self.history) >= 10:
#             self.history.pop()
        
#         self.history.append(stepArray)