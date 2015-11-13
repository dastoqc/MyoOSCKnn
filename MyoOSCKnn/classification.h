#ifndef DEF_CLASSIFICATION
#define DEF_CLASSIFICATION
class classification
{
public:
	classification();
	void finalize_python();
	int classify_rf(int emg[], int imu[]);
	void record_data_in_python(int emg[], int imu[], int classe_being_recorded);
	void train_classifier();
	void reset_training_vector();
};
#endif