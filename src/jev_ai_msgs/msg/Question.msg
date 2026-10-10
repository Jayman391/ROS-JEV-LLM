# One typed decision, answered as a categorical distribution over ordered classes.
uint8 INTERFACE_VERSION=1

uint8 NOUL=0
uint8 CHOICE=1
uint8 SCORE=2

string name
uint8 type
string instructions
string[] classes                 # CHOICE options / SCORE levels (ordered) / NOUL ["true","false"]
string[] criteria                # aligned with classes; "" = none
