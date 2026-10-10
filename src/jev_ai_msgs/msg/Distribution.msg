# Categorical answer to one Question.
uint8 INTERFACE_VERSION=1

string question
uint8 type                       # Question.NOUL | CHOICE | SCORE
string[] classes
float64[] probabilities          # aligned with classes, sums to 1
float64 confidence               # framework c = 1 - H(p)/log|A|, same meaning for all types
float64 source_confidence        # what the model itself reported (NaN if none)
