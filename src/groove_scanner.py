#!/usr/bin/env python3

from typing import List
import numpy as np
import statistics

'''
Module providing functions to analyze readings from LiDAR and to point out
the position in laser readings which represents the rail groove
'''

class GrooveScanner:
    def __init__(self, groove_depth: float) -> None:
        self._groove_depth = groove_depth

    def search_for_grooves(self, laser_readings: list) -> None:
        '''
        This function is responsible for grooves detection. Verifies whether the difference between lidar readings and the approximating function is bigger than the deviation threshold. When it occurs, the index of the groove is appended to the grooves indexes list.
        '''    
        x_vector = np.array(range(len(laser_readings)))
        laser_readings_array = np.array(laser_readings)
        
        fitting_polynomial_degree = 10
        approx_function_coefficients = np.polyfit(x_vector, laser_readings_array, fitting_polynomial_degree)
        approx_function = np.poly1d(approx_function_coefficients)
        
        self.potential_grooves_indexes = [x for x in x_vector if abs(approx_function(x) - laser_readings_array[x]) >= self._groove_depth]

    def filter_multiple_matches(self) -> List[int]:
        '''
        This function points out the approximated index of left and right groove.
        '''
        temp_list, last_groove_index = [[]], None
        for groove_index in self.potential_grooves_indexes:
            if last_groove_index is None or abs(last_groove_index - groove_index) <= 20:
                temp_list[-1].append(groove_index)
            else:
                temp_list.append([groove_index])
            last_groove_index = groove_index

        first_groove_indexes = temp_list[0]        
        try:
            second_groove_indexes = temp_list[1]
        except:
            second_groove_indexes = []

        self._first_groove_index = self._calculate_mean_index(first_groove_indexes)
        self._second_groove_index = self._calculate_mean_index(second_groove_indexes)

    @staticmethod    
    def _calculate_mean_index(groove_indexes: List[int]) -> int:
        if not groove_indexes:
            return 0
        return int(statistics.mean(groove_indexes))
    
    @property
    def get_first_groove_index(self) -> int:
        return self._first_groove_index
    
    @property
    def get_second_groove_index(self) -> int:
        return self._second_groove_index
