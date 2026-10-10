# Ground truth for calibration.
uint8 INTERFACE_VERSION=1

std_msgs/Header header
string run_id
string subject
string question
string true_class
uint64 decision_id               # 0 = join by subject + time window
string source
