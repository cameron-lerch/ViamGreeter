# ViamGreeter
This repository contains all of the code for the Viam at home technicnical exercise, an autonomous greeting robot for their event booth. The autonomous greeter utilizes a webcam to detect an person paired with a robotic arm to wave at that person and motion for them to come check out the booth. 

## Problem Statement:
### It's day two of a trade show. The booth staff have been standing for nine hours, their voices are gone, and the coffee ran out at noon. Meet Greeter: a robotic arm at the front of the booth that notices people walking by, turns to acknowledge them, gives them a moment of attention, and then moves on to the next passerby so the humans can sit down for a minute.

## Camera Details
### Here, I am assuming that I've placed a webcam directly at the base of the robotic arm, oriented along the +x direction. I chose to keep the camera mounted to a solid surface rather than somewhere along the arm to enable the system to silmutaneously greet someone and look for the next person person. Additionally, this offers a lightweight solution that allows us to achieve greeting without needing to build a complicated sceene that maintains and tracks every individual while the webacm is moving. 
