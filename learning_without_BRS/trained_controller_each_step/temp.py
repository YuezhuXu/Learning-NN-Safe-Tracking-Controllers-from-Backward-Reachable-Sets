# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 16:34:41 2026

@author: 22384
"""

import os

directory = './'  # Change this to your folder path
suffix_to_remove = '_box.mat'
new_suffix = '.mat'

for filename in os.listdir(directory):
    if filename.endswith(suffix_to_remove):
        # Create the new filename by replacing the suffix
        new_name = filename.replace(suffix_to_remove, new_suffix)
        
        # Perform the rename
        os.rename(os.path.join(directory, filename), 
                  os.path.join(directory, new_name))
        print(f"Renamed: {filename} -> {new_name}")