Main Scripts for "Machine Learning Control for HRC":

	* MBRL_Controller.py:
		This python script should run first in a virtual environment with required libraries installed. It loads the trained neural network models and it subscribes to wrench, joint_position_velocity (syncronized) and uses these information as an input to Cross-entropy-method optimization together with NN models find the best parameters. And then it publishes the optimal parameters to "/iiwa/state/KD".
	
	* iiwa_MBRL_interface.cpp:
		This C++ script can be used both for collecting data and also for running the HRC with optimal values. It subscribes to wrench (for calculating human_input_force), and KD (optimal impedance values-can be used online or averaged offline and then used). After estimating the unkonw weight it sets the set-point of cartesian impedance controller according to the human-input force (while compensating for weight). Whenever there's a human-input-force it publishes a signal to inform other scripts about the HRC (for recording the data in another script)

	* iiwa_variable_impedance_data_logger.cpp:
		This C++ script is used for logging the data during the HRC into a ".txt" file which is then used by another script for cleaning, normalizing the data and then used for training the neural network dynamic models. 

	* Paper_Z_Data_MBRL.ipynb:
		This is a python jupyter-notebook file (should run in a virtual environment with required libraries installed) which is used for cleaning, normalizing and training the neural network dynamics models for HRC which is then saved and later loaded and used by "MBRL_Controller.py" script to find the optimal values online.

	* Evaluation_Paper.ipynb:
		This is a python jupyter-notebook file (should run in a virtual environment with required libraries installed) which is used for plotting the results (physical and subjective measures)
