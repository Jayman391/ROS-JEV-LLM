# Everything needed to replay, calibrate and evaluate (OPE) one question of one decision.
uint8 INTERFACE_VERSION=1

string question
string[] classes
float64[] p_jev
float64 jev_confidence           # framework c
float64[] q_llm                  # empty if no augmentation
float64 llm_age_s
float64 beta
bool[] masked
string required                  # "" or the forced class
float64[] logits                 # final z (FRAMEWORK §6), pre-temperature
float64 tau
float64 kappa
float64[] noise                  # Gumbel g per class
bool group_noise                 # coupling draw used the group noise
string selected
float64 propensity               # P(selected | logits, tau, kappa), for OPE
bool irreversible_path           # chosen via the deterministic irreversible rule
