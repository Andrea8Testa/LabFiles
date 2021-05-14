#!/usr/bin/python2.7
from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import rospy
import message_filters
import std_msgs
#from sensor_msgs.msg import JointState
#from std_msgs.msg import Float64
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped
#from franka_core_msgs.msg import EndPointState, JointCommand, RobotState

import numpy as np
from collections import OrderedDict
from pytictoc import TicToc
#import quaternion
import torch
import torch.nn as nn
import pickle

#Import libraries. Presenti sia quelle necessarie per la costruzione della rete neurale che quelle per ROS


""" My code for calculating manipulability in the desired direction """
# Definition of the model neural network class
class NN_model(nn.Module):
    
    # Size: The number of nodes in the model.
    # Width: The number of nodes in a specific layer.
    # Depth: The number of layers in a neural network.
    def __init__(self, input_size, hidden_depth, hidden_size, output_size, print_NN=False):
        super(NN_model, self).__init__()
        
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.hidden_depth = hidden_depth
        self.output_size = output_size
        self.print_NN = print_NN
        
        self.layers = OrderedDict()
        # first layer linear part:
        # Applies a linear transformation to the incoming data: y=xAT+b
        self.layers["lin" + str(1)] = nn.Linear(self.input_size, self.hidden_size)
        # Layers appears to be a dictionary and we associate to the key lin1 the nn.Linear value
        # first layer ReLU part:
        self.layers["relu" + str(1)] = nn.ReLU()
        # Applies the rectified linear unit function element-wise:
        
        
        # other inner layers linear part:
        for i in range(2, self.hidden_depth + 1):
            # During training, randomly zeroes some of the elements of the input tensor with probability p
            self.layers["drop"+ str(i)] = nn.Dropout(p=0.2)
            self.layers["lin" + str(i)] = nn.Linear(self.hidden_size, self.hidden_size)
            self.layers["relu" + str(i)] = nn.ReLU()
            
        # last layer just linear:
        self.layers["drop"+ str(i)] = nn.Dropout(p=0.1)
        self.layers["lin" + str(self.hidden_depth +1)] = nn.Linear(self.hidden_size, self.output_size)
        
        self.pipe = nn.Sequential(self.layers)
        # A sequential container. Modules will be added to it in the order they are passed in the constructor.
        # Alternatively, an ordered dict of modules can also be passed in.
        
        if self.print_NN:
            print(self.pipe)
        
    
    def get_parameters(self):
        return self.pipe.parameters()
        
    
    def forward(self, x):
        return self.pipe(x)

# Define actor model
class ActorNN(nn.Module):
    
    def __init__(self):
        super(ActorNN, self).__init__()

        self.layers = OrderedDict()
        self.layers["lin" + str(1)] = nn.Linear(3, 64)
        self.layers["relu" + str(1)] = nn.ReLU()
        self.layers["drop"+ str(2)] = nn.Dropout(p=0.1)
        self.layers["lin" + str(2)] = nn.Linear(64,64)
        self.layers["relu" + str(2)] = nn.ReLU()
        self.layers["drop"+ str(3)] = nn.Dropout(p=0.1)
        self.layers["lin" + str(3)] = nn.Linear(64,2)
        self.layers["tanh" + str(3)] = nn.Tanh()
        
        self.pipe = nn.Sequential(self.layers)

    def get_parameters(self):
        return self.pipe.parameters()
        
    
    def forward(self, x):
        return self.pipe(x)

# Define critic model
class CriticNN(nn.Module):
    
    def __init__(self):
        super(CriticNN, self).__init__()
        self.layers = OrderedDict()
        self.layers["lin" + str(1)] = nn.Linear(5, 64)
        self.layers["relu" + str(1)] = nn.ReLU()
        self.layers["drop"+ str(2)] = nn.Dropout(p=0.1)
        self.layers["lin" + str(2)] = nn.Linear(64,64)
        self.layers["relu" + str(2)] = nn.ReLU()
        self.layers["drop"+ str(3)] = nn.Dropout(p=0.1)
        self.layers["lin" + str(3)] = nn.Linear(64,1)
         
        self.pipe = nn.Sequential(self.layers)

    def get_parameters(self):
        return self.pipe.parameters()
        
    
    def forward(self, x):
        return self.pipe(x)

class Q_LMPC():
    
    def __init__(self, ensemble_NN_, actor_NN_, critic_NN_, ensemble_size_, prediction_horizon_, samples_num_):
        
        self.NN_ensemble = ensemble_NN_
        self.actor_NN = actor_NN_
        self.critic_NN = critic_NN_
        self.ensemble_size = ensemble_size_
        self.prediction_horizon = prediction_horizon_
        self.samples_num = samples_num_
        
        cartesian_position_sub = message_filters.Subscriber("/franka_ee_pose", PoseStamped, queue_size = 1, buff_size = 10)
        cartesian_velocity_sub = message_filters.Subscriber("/franka_ee_velocity", TwistStamped, queue_size = 1, buff_size = 10)
        wrench_sub = message_filters.Subscriber("/franka_ee_wrench", WrenchStamped, queue_size = 1, buff_size = 10)
        # Subscription to a certain topic and message type

        sync = message_filters.ApproximateTimeSynchronizer([cartesian_position_sub, cartesian_velocity_sub, wrench_sub], queue_size = 1, slop = 0.1 )
        #policy used by message_filters::sync::Synchronizer to match messages coming on a set of topics
        sync.registerCallback(self.measurements_callback)
        #In the ROS setting a callback in most cases is a message handler. You define the message handler function and give it to subscribe.
        #You never call it yourself, but whenever a message arrives ROS will call you message handler and pass it the new message,
        #so you can deal with that.
        self.u_pub = rospy.Publisher("/QLMPC_pose", geometry_msgs.msg.PoseStamped, queue_size = 1)
        self.D_pub = rospy.Publisher("/D_information", std_msgs.msg.Float64, queue_size = 1)
        # Publishing a message on a certain topic
        self.tictoc = TicToc()
        
        # load data
        a_file = open("data.pkl", "rb")
        self.training_dict = pickle.load(a_file)
        #self.training_dict = torch.load('tensors.pt')
        self.x_mean_v, self.x_std_v, self.y_mean_v, self.y_std_v = self.training_dict['xy_norm']
        self.xn_train = self.training_dict['xn_train']
        self.yn_train = self.training_dict['yn_train']
        
        # Definition of the limits
        self.us = 0.5
        self.ul = -0.5
        self.action_mean = (self.us + self.ul)/2
        self.action_std = (self.us - self.ul)/2
        self.action_norm = (self.action_mean, self.action_std)

        self.Drs = 5
        self.Drl = 0.1
        self.damping_mean = (self.Drs + self.Drl)/2
        self.damping_std = (self.Drs - self.Drl)/2
        self.damping_norm = (self.damping_mean, self.damping_std)

        self.fh_mean = np.mean(self.x_mean_v[2:3])
        self.fh_std  = np.std(self.x_std_v[2:3])
        self.fh_norm = (self.fh_mean, self.fh_std)
        
        self.Cost_norm = np.load('Cost.npy')
        self.media_costo, self.stad_cost = self.Cost_norm
        
       
        # putting it in delta form for learning
        ### New data (st, at)--> (st+1 - st)
        self.yn_train_d = self.yn_train[:,0:3] - self.xn_train[:,0:3]
       
        self.z_record_norm = np.zeros((self.prediction_horizon + 1, 1))
        self.x_record_norm = np.zeros((self.prediction_horizon + 1, 1))
        self.fh_record_norm = np.zeros((self.prediction_horizon + 1, 1))
        self.u_record_norm = np.zeros((self.prediction_horizon, 1))
        self.D_record_norm = np.zeros((self.prediction_horizon, 1))
        self.iter_buffer_ = 0
        self.rate = rospy.Rate(5)  # 5 Hz
    
    # definition of cost function
    def cost_func_p(self, u_, fh_): 
      
        N_  = u_.shape[1]   # The samples are the values extracted  
        ense_ = u_.shape[0] # 5
        
        # weights
        Q_ = 100
        R_ = 0.01
        
        cost_ = ((np.sum(np.sum(np.multiply(fh_,fh_)*Q_, axis=2).reshape(ense_, N_), axis=0)/ense_).reshape(N_) +
                 (np.sum(np.sum(np.multiply(u_,u_)*R_, axis=2).reshape(ense_, N_), axis=0)/ense_).reshape(N_)) 
        
        return cost_

    
    #definition of Cross Entropy Method
    def CEM_norm_p(self, x_initial_, action_dim_, time_horizon_, num_samples_, xy_norm_, NN_model_, num_ensembles_cem_):
    
        assert(x_initial_.shape[0] == 1)  # state should be a row vector z, x and fh (no actions)
        # if this condition is not true, an assertion error is returned
        
        x_mean_v_, x_std_v_, y_mean_v_, y_std_v_ = xy_norm_ # normalization variables
        
        state_dim_  = x_initial_.shape[1]
        state_action_dim_ = action_dim_ + state_dim_ + 1
        smoothing_rate_ = 0.9 #0.9
        iteration_      = 10 #10
        num_elites_ = 32 #32
        # 16-32 elites are enough with 64-128 samples for: action_dim * time_horizon <= 100
        num_ensembles_ = num_ensembles_cem_
        
        for k in range(num_ensembles_):
            NN_model_["NN"+str(k)].to("cpu") 
        
        # Initializing:
        mu_matrix_u_  = np.zeros((action_dim_, time_horizon_))
        std_matrix_u_ = np.ones((action_dim_, time_horizon_))
        
        mu_matrix_D_  = np.zeros((1, time_horizon_))
        std_matrix_D_ = np.ones((1, time_horizon_))
        
        for _ in range(iteration_):
    
            state_t_broadcasted_ = np.ones((num_ensembles_, num_samples_, state_dim_)) * x_initial_
    
            if 'action_samples_' in locals(): 
                del action_samples_
                del damping_samples_
    
            # Draw random samples from a normal (Gaussian) distribution.
            action_samples_ = np.random.normal(loc=mu_matrix_u_, scale=std_matrix_u_,
                                               size=(num_samples_, action_dim_, time_horizon_))
            
            damping_samples_ = np.random.normal(loc=mu_matrix_D_, scale=std_matrix_D_,
                                               size=(num_samples_, 1, time_horizon_))
            # it returns an array of dimensions: (64, 1, 5)
            # the values higher or lower than a threshold are 
            action_samples_[action_samples_ >=  1] =  1
            action_samples_[action_samples_ <= -1] = -1
            #print(action_samples_)
            damping_samples_[damping_samples_ >=  1] =  1
            damping_samples_[damping_samples_ <= -1] = -1
    
            costs_ = np.zeros(num_samples_)
    
            # Evaluate the trajectories and find the elites
            for t in range(time_horizon_):
    
                action_t_norm_ = action_samples_[:,:,t].reshape(num_samples_, action_dim_)
                # 2 dimensions
                action_t_broadcasted_norm_ = np.ones((num_ensembles_, num_samples_, action_dim_)) * action_t_norm_
                # 3 dimensions
                
                damping_t_norm_ = damping_samples_[:,:,t].reshape(num_samples_, 1)
                # 2 dimensions
                damping_t_broadcasted_norm_ = np.ones((num_ensembles_, num_samples_, 1)) * damping_t_norm_
                # 3 dimensions
                
                state_t_broadcasted_norm_ = (state_t_broadcasted_ - x_mean_v_[0:state_dim_])/x_std_v_[0:state_dim_]
                state_action_norm_ = np.append(state_t_broadcasted_norm_, action_t_broadcasted_norm_, axis=2)
                state_action_damping_norm_ = np.append(state_action_norm_, damping_t_broadcasted_norm_, axis=2)
                # at this point the dimension is: (5 ensembles, 64 samples, 5 numbers (z, x, fh, u and Dr))
                
                state_action_damping_norm_torch_ = torch.tensor(state_action_damping_norm_, dtype=torch.float32, device="cpu")
                state_t_broadcasted_norm_torch_ = torch.tensor(state_t_broadcasted_norm_, 
                                                               dtype=torch.float32, device="cpu")    
                NN_model_["NN0"].eval()
                NN_model_["NN1"].eval()
                NN_model_["NN2"].eval()
                NN_model_["NN3"].eval()
                NN_model_["NN4"].eval()
    
                state_tt_norm_torch_ = NN_model_["NN0"].forward(state_action_damping_norm_torch_[0,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[0,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )
                state_tt_norm_torch_ = torch.cat((state_tt_norm_torch_, NN_model_["NN1"].forward(state_action_damping_norm_torch_[1,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[1,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )), dim=0)
                state_tt_norm_torch_ = torch.cat((state_tt_norm_torch_, NN_model_["NN2"].forward(state_action_damping_norm_torch_[2,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[2,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )), dim=0)
                state_tt_norm_torch_ = torch.cat((state_tt_norm_torch_, NN_model_["NN3"].forward(state_action_damping_norm_torch_[3,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[3,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )), dim=0)
                state_tt_norm_torch_ = torch.cat((state_tt_norm_torch_, NN_model_["NN4"].forward(state_action_damping_norm_torch_[4,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[4,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )), dim=0)
    
                state_tt_norm_ = np.asarray(state_tt_norm_torch_.detach()).reshape(num_ensembles_, num_samples_, 
                                                                                   state_dim_)
                state_tt_ = state_tt_norm_*y_std_v_ + y_mean_v_ # not normalized
                #print (state_tt_[:,:,2:3].shape)
                action_mean_forcost_ = np.asarray(x_mean_v_[3:4])
                action_std_forcost_  = np.asarray(x_std_v_[3:4])
                action_forcost_ = action_t_broadcasted_norm_*action_std_forcost_ + action_mean_forcost_
                    
                step_cost_ = self.cost_func_p(action_forcost_,state_tt_[:,:,2:3])
                # fh and u are taken in the cost without normalization
                state_t_broadcasted_ = state_tt_
                del state_action_damping_norm_torch_; del state_t_broadcasted_norm_torch_; del action_forcost_
                torch.cuda.empty_cache()
    
                costs_ += step_cost_
                
            top_elites_index_ = costs_.argsort()[::1][:num_elites_]  # sorting index with min cost first
    
            elites_u_  = action_samples_[top_elites_index_,:,:].reshape(num_elites_, action_dim_, time_horizon_)
            mu_matrix_new_u_  = np.sum(elites_u_, axis=0)/num_elites_
            std_matrix_new_u_ = np.sqrt( np.sum( np.square(elites_u_ - mu_matrix_new_u_), axis=0)/num_elites_) 
            
            elites_D_ = damping_samples_[top_elites_index_,:,:].reshape(num_elites_, 1, time_horizon_)
            mu_matrix_new_D_  = np.sum(elites_D_, axis=0)/num_elites_
            std_matrix_new_D_ = np.sqrt( np.sum( np.square(elites_D_ - mu_matrix_new_D_), axis=0)/num_elites_) 
            # mu_new should broadcast to size of elites_ then subtract and then elementwise square 
    
            # Update the mu_ and std_
            mu_matrix_u_  = smoothing_rate_*mu_matrix_new_u_  + (1-smoothing_rate_)*mu_matrix_u_
            std_matrix_u_ = smoothing_rate_*std_matrix_new_u_ + (1-smoothing_rate_)*std_matrix_u_
            best_action_n_seq_ = elites_u_[0,:,:].reshape(action_dim_, time_horizon_)
            
            mu_matrix_D_  = smoothing_rate_*mu_matrix_new_D_  + (1-smoothing_rate_)*mu_matrix_D_
            std_matrix_D_ = smoothing_rate_*std_matrix_new_D_ + (1-smoothing_rate_)*std_matrix_D_
            best_damping_n_seq_ = elites_D_[0,:,:].reshape( 1, time_horizon_)
            
            action_mean_v_ = np.asarray(x_mean_v_[3:4]).reshape(action_dim_,1)
            action_std_v_  = np.asarray(x_std_v_[3:4]).reshape(action_dim_,1)
            damping_mean_v_ = np.asarray(x_mean_v_[4:5]).reshape(1,1)
            damping_std_v_  = np.asarray(x_std_v_[4:5]).reshape(1,1)
            best_action_seq_ = best_action_n_seq_*action_std_v_ + action_mean_v_
            best_damping_seq_ = best_damping_n_seq_*damping_std_v_ + damping_mean_v_
            # mu is the average of the source from which samples are generated, but the real normalization for
            # the action applied and used in the cost function is made respect to the previous average function
            
        
        return best_action_seq_, best_damping_seq_
    
    def model_approximator_train(self, state_force_norm_train, u_norm_train, D_norm_train, NN_model, Nh_, learning_rate):
        
        x_data_ = np.copy(state_force_norm_train[0:Nh_, :])
        x_data_ = np.append(x_data_, u_norm_train, axis = 1)
        x_data_ = np.append(x_data_, D_norm_train, axis = 1)
        y_data_ = state_force_norm_train[1:Nh_+1, :] - state_force_norm_train[0:Nh_, :]
        
        
        NN_model_    = NN_model
        learning_rate_   = learning_rate
        
        
        NN_model_.to("cpu")  # putting the model into GPU (but not available now)
        cost_ = nn.MSELoss()  # Mean squared loss
        # Creates a criterion that measures the mean squared error (squared L2 norm) between each element 
        # in the input xxx and target yyy 
        optimizer_ = torch.optim.Adam(NN_model_.get_parameters(), lr =learning_rate_)
        NN_model_.train()
        
        # clear grads
        optimizer_.zero_grad()
    
        # Forward propagation
        x_ = torch.tensor(x_data_, dtype= torch.float32, device="cpu")
        y_ = torch.tensor(y_data_, dtype= torch.float32, device="cpu")
    
        y_hat_ = NN_model_.forward(x_)
        loss_ = cost_.forward(y_hat_, y_)              # calculating the loss
        loss_.backward()                              # backprop
        optimizer_.step()                             # updating the parameters
        torch.cuda.empty_cache()
               
            
        return

        # Input data are normalized
    def Critic_train(self, state_force_train, action_train, damping_train, Actor_NN, Critic_NN, xy_norm, Cost_norm, Nh, learning_rate):
        
        x_mean_v, x_std_v, y_mean_v, y_std_v = xy_norm
        fh_mean_ = x_mean_v[2]
        fh_std_ = x_std_v[2]
        u_mean_ = x_mean_v[3]
        u_std_ = x_std_v[3]
        cost_mean, cost_std_dev = Cost_norm
        
        Actor_NN_ = Actor_NN
        Critic_NN_ = Critic_NN
        Nh_ = Nh
        learning_rate_ = learning_rate
        
        state_force_train_ = np.copy(state_force_train[0:Nh_,:])
        state_force_trainp1_ = np.copy(state_force_train[1:Nh_+1,:])
        action_train_ = np.copy(action_train[0:Nh_])
        damping_train_ = np.copy(damping_train[0:Nh_])
        
        actor_eval_ = np.copy(state_force_trainp1_)
        critic_train_ = np.append(state_force_train_, action_train_, axis=1)
        critic_train_ = np.append(critic_train_, damping_train_, axis=1)
        critic_trainp1_ = np.append(state_force_trainp1_, action_train_, axis=1) # column 3 to update
        critic_trainp1_ = np.append(critic_trainp1_, damping_train_, axis=1) # column 4 to update
    
        Actor_NN_.to("cpu")  # putting the model into GPU (but not available now)
        Critic_NN_.to("cpu")
        #optimizer_C = torch.optim.Adam(Critic_NN_.layers["lin" + str(2)].parameters(), lr =learning_rate_)
        optimizer_C = torch.optim.Adam(Critic_NN_.get_parameters(), lr =learning_rate_)
        errore_f = nn.MSELoss()
        
        actor_data_torch_ = torch.tensor(actor_eval_, dtype=torch.float32, device="cpu")
        critic_data_torch_ = torch.tensor(critic_train_, dtype=torch.float32, device="cpu")
        critic_data_torchp1_ = torch.tensor(critic_trainp1_, dtype=torch.float32, device="cpu")
        
        for j in range(0, Nh_):
            
            Actor_NN_.eval()
            
            # actor network to estimate u from state and human force at step n+1
            U_npiu1, Dr_npiu1 = Actor_NN_.forward(actor_data_torch_[j,:])
            # Update of the fourth column
            critic_data_torchp1_[j,3] = U_npiu1
            critic_data_torchp1_[j,4] = Dr_npiu1
            
            Critic_NN_.train()
            optimizer_C.zero_grad()
            
            # estimation of Qn from state, human force and input at step n
            # Q represent the cost function integrated between t and infinite
            Q_n = Critic_NN_.forward(critic_data_torch_[j,:])#*cost_std_dev*5 + cost_mean*5
            
            # estimation of Qn+1 from state and human force at step n+1 and from estimated input at step n+1
            Q_npiu1 = Critic_NN_.forward(critic_data_torchp1_[j,:])#*cost_std_dev*5 + cost_mean*5
            # The output of a neural network is always normalized. Since we compare it with a not normalized cost, 
            # it is necessary to normalize it. 
            print("Input Q_n", critic_data_torch_[j,:])
            print("Input Q_npiu1", critic_data_torchp1_[j,:])
            
            # Bellman equation
            
            # the cost function requires 3d arrays, so we need to convert the 2d tensors we have
            # besides the input of this function are normalized, however we need to enter in the cost function
            # with the non normalized values.
            Cost_f_0 = (critic_data_torch_[j:j+1,2:3].unsqueeze(0)).detach().numpy()*fh_std_ + fh_mean_
            Cost_u_0 = (critic_data_torch_[j:j+1,3:4].unsqueeze(0)).detach().numpy()*u_std_ + u_mean_
            
            print("U", Cost_u_0)
            print("F", Cost_f_0)
            
            COSTO = (self.cost_func_p(Cost_u_0, Cost_f_0)- cost_mean)/cost_std_dev # normalization 
            del Cost_u_0, Cost_f_0
            
            Q_Bellman = Q_npiu1 + torch.from_numpy(COSTO)
            print("Costo ", COSTO)
            print("Q_n ", Q_n)
            print("Q_npiu1", Q_npiu1)
            print("Q_Bellman", Q_Bellman)
            # Cost function
            Error_c = errore_f.forward(Q_n, Q_Bellman)
            
            Error_c.backward(retain_graph=True )                          # backprop
            optimizer_C.step()                             # updating the parameters  
        
        return
    
    def ComputationUP(self, xy_norm, state_action_norm_torch_lie, model_approximator, num_ensembles = 5):
    
        x_mean_v, x_std_v, y_mean_v, y_std_v = xy_norm
    
        state_action_norm_torch_lie_f = torch.clone(state_action_norm_torch_lie)
        state_action_norm_torch_lie_f[:,3:4] = 0
        state_action_norm_torch_lie_f[:,4:5] = 0
        Dati_lie_f = torch.zeros(2,5)
        for k in range(num_ensembles):
            model_approximator["NN"+str(k)].to("cpu")
            model_approximator["NN"+str(k)].eval()
            output_f_lie = model_approximator["NN"+str(k)].forward(state_action_norm_torch_lie_f)
            Dati_lie_f[:,k] = output_f_lie[:,0:2]
        #Risultato_lie_f = (torch.sum(Dati_lie_f, dim=1)/num_ensembles)
        Risultato_lie_f = (torch.sum(Dati_lie_f, dim=1)/num_ensembles)*torch.FloatTensor(x_std_v[0:2]) + torch.FloatTensor(x_mean_v[0:2])
        #print( Risultato_lie_f )
    
        state_action_norm_torch_lie_g1 = torch.clone(state_action_norm_torch_lie)
        state_action_norm_torch_lie_g1[:,3:4] = 1
        state_action_norm_torch_lie_g1[:,4:5] = 0
        Dati_lie_g1 = torch.zeros(2,5)
        for k in range(num_ensembles):
            model_approximator["NN"+str(k)].to("cpu")
            model_approximator["NN"+str(k)].eval()
            output_g1_lie = model_approximator["NN"+str(k)].forward(state_action_norm_torch_lie_g1)
            Dati_lie_g1[:,k] = output_g1_lie[:,0:2]
        #Risultato_lie_g1 = (torch.sum(Dati_lie_g1, dim=1)/num_ensembles)
        Risultato_lie_g1 = (torch.sum(Dati_lie_g1, dim=1)/num_ensembles)*torch.FloatTensor(x_std_v[0:2]) + torch.FloatTensor(x_mean_v[0:2]) - Risultato_lie_f
        #print("Risultato_lie_g1", Risultato_lie_g1 )
        
        state_action_norm_torch_lie_g2 = torch.clone(state_action_norm_torch_lie)
        state_action_norm_torch_lie_g2[:,3:4] = 0
        state_action_norm_torch_lie_g2[:,4:5] = 1
        Dati_lie_g2 = torch.zeros(2,5)
        for k in range(num_ensembles):
            model_approximator["NN"+str(k)].to("cpu")
            model_approximator["NN"+str(k)].eval()
            output_g2_lie = model_approximator["NN"+str(k)].forward(state_action_norm_torch_lie_g2)
            Dati_lie_g2[:,k] = output_g2_lie[:,0:2]
        #Risultato_lie_g2 = (torch.sum(Dati_lie_g2, dim=1)/num_ensembles)
        Risultato_lie_g2 = (torch.sum(Dati_lie_g2, dim=1)/num_ensembles)*torch.FloatTensor(x_std_v[0:2]) + torch.FloatTensor(x_mean_v[0:2]) - Risultato_lie_f
        #print("Risultato_lie_g2", Risultato_lie_g2 )
        
        # Vl = 1/2 z P1 z + 1/2 (xd-x)P2(xd-x)
        P1 = 1e-5 #1e-5 peso presentazione     #7
        P2 = 4e-4 #1e-4 peso presentazione  #-700
        gradient_P_z_lie = P1*(state_action_norm_torch_lie[:,0:1]*torch.FloatTensor(x_std_v[0:1]) + torch.FloatTensor(x_mean_v[0:1]))
        gradient_P_x_lie = -P2*((state_action_norm_torch_lie[:,2:3])*torch.FloatTensor(x_std_v[2:3]) + torch.FloatTensor(x_mean_v[2:3]))
        #print("gradient_P_z_lie: ", gradient_P_z_lie)
        #print("gradient_P_x_lie: ", gradient_P_x_lie)
    
        gradient_P = torch.cat((gradient_P_z_lie, gradient_P_x_lie), 0)
        Lie_F = Risultato_lie_f.unsqueeze_(0).T
        Lie_G1 = Risultato_lie_g1.unsqueeze_(0).T
        Lie_G2 = Risultato_lie_g2.unsqueeze_(0).T
        #print("gradient_P: ", gradient_P[0,:], gradient_P.shape)
        #print("Lie_g: ", Lie_F[0,:], Lie_F.shape)
        Lf_P = torch.sum(gradient_P[0,:]*Lie_F[0,:] + gradient_P[1,:]*(state_action_norm_torch_lie[:,0:1]*torch.FloatTensor(x_std_v[0:1]) + torch.FloatTensor(x_mean_v[0:1])))
        Lg1_P = torch.sum(gradient_P[0,:]*Lie_G1[0,:] + gradient_P[1,:]*torch.abs(Lie_G1[1,:])) # gradient_P[0,:]*Lie_G1[0,:]
        Lg2_P = torch.sum(gradient_P[:,:]*Lie_G2[:,:])
        a = Lf_P
        b1 = Lg1_P
        b2 = Lg2_P
        beta = b1**2 + b2**2
        #print("Lg_P: ", Lg_P)
        #print("Lf_P: ", Lf_P)
    
    
        h1 = ((-(a + torch.sqrt(a**2 + beta**2))/beta)*b1).item()
        h2 = ((-(a + torch.sqrt(a**2 + beta**2))/beta)*b2).item()
    
        if h1 > 0.1: # 0.25
            h1 = 0.1
        elif h1 < -0.1: # -0.25
            h1 = -0.1
    
        if h2 > 3: #18
            h2 = 3 
        elif h2 < 2: #12
            h2 = 2
        return h1, Lg1_P.item(), h2, Lg2_P.item()
    
    
    
    def CEM_critic(self, x_initial_, action_dim_, time_horizon_, num_samples_, xy_norm_, Cost_norm, NN_model_, Critic_NN_, num_ensembles_cem_):
    
        assert(x_initial_.shape[0] == 1)  # state should be a row vector z, x and fh (no actions)
        # if this condition is not true, an assertion error is returned
        
        x_mean_v_, x_std_v_, y_mean_v_, y_std_v_ = xy_norm_ # normalization variables
        cost_mean, cost_std_dev = Cost_norm
        
        state_dim_  = x_initial_.shape[1]
        state_action_dim_ = action_dim_ + state_dim_ + 1
        smoothing_rate_ = 0.9 #0.9
        iteration_      = 5 #10
        num_elites_ = 16 #32
        # 16-32 elites are enough with 64-128 samples for: action_dim * time_horizon <= 100
        num_ensembles_ = num_ensembles_cem_
        
        for k in range(num_ensembles_):
            NN_model_["NN"+str(k)].to("cpu") 
        
        # Initializing:
        mu_matrix_u_  = np.zeros((action_dim_, time_horizon_))
        std_matrix_u_ = np.ones((action_dim_, time_horizon_))
        
        mu_matrix_D_  = np.zeros((1, time_horizon_))
        std_matrix_D_ = np.ones((1, time_horizon_))
        
        for _ in range(iteration_):
    
            state_t_broadcasted_ = np.ones((num_ensembles_, num_samples_, state_dim_)) * x_initial_
    
            if 'action_samples_' in locals(): 
                del action_samples_
                del damping_samples_
    
            # Draw random samples from a normal (Gaussian) distribution.
            action_samples_ = np.random.normal(loc=mu_matrix_u_, scale=std_matrix_u_,
                                               size=(num_samples_, action_dim_, time_horizon_))
            
            damping_samples_ = np.random.normal(loc=mu_matrix_D_, scale=std_matrix_D_,
                                               size=(num_samples_, 1, time_horizon_))
            # it returns an array of dimensions: (64, 1, 5)
            # the values higher or lower than a threshold are 
            action_samples_[action_samples_ >=  1] =  1
            action_samples_[action_samples_ <= -1] = -1
            #print(action_samples_)
            damping_samples_[damping_samples_ >=  1] =  1
            damping_samples_[damping_samples_ <= -1] = -1
    
            costs_ = np.zeros(num_samples_)
            Z_function_u_ = torch.zeros(num_samples_, time_horizon_)
            Z_function_D_ = torch.zeros(num_samples_, time_horizon_)
    
            # Evaluate the trajectories and find the elites
            for t in range(time_horizon_):
    
                action_t_norm_ = action_samples_[:,:,t].reshape(num_samples_, action_dim_)
                # 2 dimensions
                action_t_broadcasted_norm_ = np.ones((num_ensembles_, num_samples_, action_dim_)) * action_t_norm_
                # 3 dimensions
                
                damping_t_norm_ = damping_samples_[:,:,t].reshape(num_samples_, 1)
                # 2 dimensions
                damping_t_broadcasted_norm_ = np.ones((num_ensembles_, num_samples_, 1)) * damping_t_norm_
                # 3 dimensions
                
                state_t_broadcasted_norm_ = (state_t_broadcasted_ - x_mean_v_[0:state_dim_])/x_std_v_[0:state_dim_]
                state_action_norm_ = np.append(state_t_broadcasted_norm_, action_t_broadcasted_norm_, axis=2)
                state_action_damping_norm_ = np.append(state_action_norm_, damping_t_broadcasted_norm_, axis=2)
                # at this point the dimension is: (5 ensembles, 64 samples, 5 numbers (z, x, fh, u and Dr))
                
                state_action_damping_norm_torch_ = torch.tensor(state_action_damping_norm_, dtype=torch.float32, device="cpu")
                state_t_broadcasted_norm_torch_ = torch.tensor(state_t_broadcasted_norm_, 
                                                               dtype=torch.float32, device="cpu")
                NN_model_["NN0"].eval()
                NN_model_["NN1"].eval()
                NN_model_["NN2"].eval()
                NN_model_["NN3"].eval()
                NN_model_["NN4"].eval()
    
                state_tt_norm_torch_ = NN_model_["NN0"].forward(state_action_damping_norm_torch_[0,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[0,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )
                state_tt_norm_torch_ = torch.cat((state_tt_norm_torch_, NN_model_["NN1"].forward(state_action_damping_norm_torch_[1,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[1,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )), dim=0)
                state_tt_norm_torch_ = torch.cat((state_tt_norm_torch_, NN_model_["NN2"].forward(state_action_damping_norm_torch_[2,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[2,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )), dim=0)
                state_tt_norm_torch_ = torch.cat((state_tt_norm_torch_, NN_model_["NN3"].forward(state_action_damping_norm_torch_[3,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[3,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )), dim=0)
                state_tt_norm_torch_ = torch.cat((state_tt_norm_torch_, NN_model_["NN4"].forward(state_action_damping_norm_torch_[4,:,:].view(num_samples_, state_action_dim_)) + state_t_broadcasted_norm_torch_[4,:,:].view(num_samples_, state_dim_).view(1,num_samples_, state_dim_ )), dim=0)
    
                state_tt_norm_ = np.asarray(state_tt_norm_torch_.detach()).reshape(num_ensembles_, num_samples_, 
                                                                                   state_dim_)
                state_tt_ = state_tt_norm_*y_std_v_ + y_mean_v_ # not normalized
                
                # Computation of the cost function
                Critic_input_ = state_action_damping_norm_torch_
                # Critic Network is introduced, a 64 variables vector is expected in output
                step_cost_ = np.empty_like(np.zeros(num_samples_))
                #print(step_cost_.shape)
                for j1 in range(0,num_samples_):
                    costi_rete_ = []
                    for j2 in range(0,num_ensembles_cem_):
                        Critic_NN_.eval()
                        #print("input_critic",Critic_input_[j2:j2+1,j1:j1+1,:])
                        costo_rete_ = Critic_NN_.forward(Critic_input_[j2:j2+1,j1:j1+1,:])
                        #print("costo_rete_",costo_rete_)
                        costi_rete_.append(costo_rete_.item())
                    step_cost_[j1] = (np.sum(costi_rete_)/num_ensembles_cem_)
                    
                STEPPO = step_cost_.reshape(num_samples_)
                #print("STEPPO", STEPPO)
                
                #print("state_action_norm_torch_", state_action_norm_torch_.shape)
                state_action_damping_norm_torch_lie_ = state_action_damping_norm_torch_.sum(0)/num_ensembles_cem_
                #print("state_action_norm_torch_lie_", state_action_norm_torch_lie_.shape)
                
                # Computation of constraint function
                Lyapunov_h1_ = torch.zeros(num_samples_,1)
                Lyapunov_c1_ = torch.zeros(num_samples_,1)
                Lyapunov_h2_ = torch.zeros(num_samples_,1)
                Lyapunov_c2_ = torch.zeros(num_samples_,1)
                Upper_limit_u_norm_ = torch.zeros(num_samples_,1)
                Lower_limit_u_norm_ = torch.zeros(num_samples_,1)
                Upper_limit_D_norm_ = torch.zeros(num_samples_,1)
                Lower_limit_D_norm_ = torch.zeros(num_samples_,1)
                for iter in range(num_samples_):
                    #print(state_action_norm_torch_lie_[iter:iter+1,:])
                    Lyapunov_h1_[iter, :], Lyapunov_c1_[iter, :], Lyapunov_h2_[iter, :], Lyapunov_c2_[iter, :]  = self.ComputationUP(xy_norm_, state_action_damping_norm_torch_lie_[iter:iter+1,:], NN_model_, num_ensembles = 5)
                    if Lyapunov_c1_[iter, :] > 0:
                        Upper_limit_u_norm_[iter, :] = (Lyapunov_h1_[iter, :] - x_mean_v_[3])/x_std_v_[3]
                        Lower_limit_u_norm_[iter, :] = -1
                    elif Lyapunov_c1_[iter, :] < 0:
                        Upper_limit_u_norm_[iter, :] = 1
                        Lower_limit_u_norm_[iter, :] = (Lyapunov_h1_[iter, :] - x_mean_v_[3])/x_std_v_[3]
                        
                    if Lyapunov_c2_[iter, :] > 0:
                        Upper_limit_D_norm_[iter, :] = (Lyapunov_h2_[iter, :] - x_mean_v_[4])/x_std_v_[4]
                        Lower_limit_D_norm_[iter, :] = -1
                    elif Lyapunov_c2_[iter, :] < 0:
                        Upper_limit_D_norm_[iter, :] = 1
                        Lower_limit_D_norm_[iter, :] = (Lyapunov_h2_[iter, :] - x_mean_v_[4])/x_std_v_[4]
    
                print("Upper_limit_u_norm_", Upper_limit_u_norm_[0:10,:].T)
                print("Lower_limit_u_norm_", Lower_limit_u_norm_[0:10,:].T)
                print("Upper_limit_D_norm_", Upper_limit_D_norm_[0:10,:].T)
                print("Lower_limit_D_norm_", Lower_limit_D_norm_[0:10,:].T)
                #print((state_action_norm_torch_lie_[:,3:4] - Upper_limit_norm_).shape)
                Matrix_Z_u_ = torch.cat( (Lower_limit_u_norm_ - state_action_damping_norm_torch_lie_[:,3:4], state_action_damping_norm_torch_lie_[:,3:4] - Upper_limit_u_norm_, torch.zeros(num_samples_,1) ), 1 )
                Matrix_Z_D_ = torch.cat( (Lower_limit_D_norm_ - state_action_damping_norm_torch_lie_[:,4:5], state_action_damping_norm_torch_lie_[:,4:5] - Upper_limit_D_norm_, torch.zeros(num_samples_,1) ), 1 )
                #print(Matrix_Z_)
                Z_function_u_[:, t:t+1], inds_u =  torch.max(Matrix_Z_u_, 1, keepdim=True)
                Z_function_D_[:, t:t+1], inds_D =  torch.max(Matrix_Z_D_, 1, keepdim=True)
                #print("Z_function_u_[t, :]: ", Z_function_u_[:, t:t+1])
                "fare somma di queste funzioni"
                
                # the input of the critic network is normalized, the output is assumed not normalized
                state_t_broadcasted_ = state_tt_
                del state_action_damping_norm_torch_; del state_t_broadcasted_norm_torch_; del step_cost_; del Matrix_Z_u_; del Matrix_Z_D_; 
                torch.cuda.empty_cache()
    
                costs_ += STEPPO
            
            #print("costi", costs_)
            
            #print("Z_function_", Z_function_)
            Z_function_ = Z_function_u_ + Z_function_D_/5
            #print("Z_function_", Z_function_)
            Z_lie_, index_lie = torch.max(Z_function_, 1, keepdim=True)
            #print("Z_lie_", Z_lie_)
            Cost_constraint_ = 2 - Z_lie_.detach().numpy()
            
            #print("costraint" , Cost_constraint_)
            
            num_feasibles_=0
            contatore_while = 0
            while num_feasibles_== 0:
                
                Index_to_take_, values = np.where( (Cost_constraint_ >= 2 - contatore_while) )
                feasibles_u_  = action_samples_[Index_to_take_,:,:].reshape(len(Index_to_take_), action_dim_, time_horizon_)
                feasibles_D_  = damping_samples_[Index_to_take_,:,:].reshape(len(Index_to_take_), 1, time_horizon_)
                feasibles_costs_ = costs_[Index_to_take_]
                num_feasibles_ = feasibles_u_.shape[0]
                if contatore_while > 2:
                    print("errore")
                    return
                contatore_while += 0.1
    
            print("num_feasibles: ", num_feasibles_)
            print("contatore_while: ", contatore_while)
            #print("feasibles_costs", feasibles_costs_)
            
    
            if num_feasibles_ < num_elites_:
                top_elites_index_ = feasibles_costs_.argsort()[::1]
                elites_u_ = feasibles_u_[top_elites_index_,:,:].reshape(num_feasibles_, action_dim_, time_horizon_)
                elites_D_ = feasibles_D_[top_elites_index_,:,:].reshape(num_feasibles_, 1, time_horizon_)
                mu_matrix_new_u_  = np.sum(elites_u_, axis=0)/num_feasibles_
                std_matrix_new_u_ = np.sqrt( np.sum( np.square(elites_u_ - mu_matrix_new_u_), axis=0)/num_feasibles_)
                mu_matrix_new_D_  = np.sum(elites_D_, axis=0)/num_feasibles_
                std_matrix_new_D_ = np.sqrt( np.sum( np.square(elites_D_ - mu_matrix_new_D_), axis=0)/num_feasibles_)
                # mu_new should broadcast to size of elites_ then subtract and then elementwise square 
            elif num_feasibles_ >= num_elites_:
                top_elites_index_ = feasibles_costs_.argsort()[::1][:num_elites_]
                elites_u_ = feasibles_u_[top_elites_index_,:,:].reshape(num_elites_, action_dim_, time_horizon_)
                elites_D_ = feasibles_D_[top_elites_index_,:,:].reshape(num_elites_, 1, time_horizon_)
                mu_matrix_new_u_  = np.sum(elites_u_, axis=0)/num_elites_
                std_matrix_new_u_ = np.sqrt( np.sum( np.square(elites_u_ - mu_matrix_new_u_), axis=0)/num_elites_)
                mu_matrix_new_D_  = np.sum(elites_D_, axis=0)/num_elites_
                std_matrix_new_D_ = np.sqrt( np.sum( np.square(elites_D_ - mu_matrix_new_D_), axis=0)/num_elites_)
                # mu_new should broadcast to size of elites_ then subtract and then elementwise square
                
    
            # Update the mu_ and std_
            mu_matrix_u_  = smoothing_rate_*mu_matrix_new_u_  + (1-smoothing_rate_)*mu_matrix_u_
            std_matrix_u_ = smoothing_rate_*std_matrix_new_u_ + (1-smoothing_rate_)*std_matrix_u_
            mu_matrix_D_  = smoothing_rate_*mu_matrix_new_D_  + (1-smoothing_rate_)*mu_matrix_D_
            std_matrix_D_ = smoothing_rate_*std_matrix_new_D_ + (1-smoothing_rate_)*std_matrix_D_
            best_action_n_seq_ = elites_u_[0,:,:].reshape(action_dim_, time_horizon_)
            best_damping_n_seq_ = elites_D_[0,:,:].reshape(1, time_horizon_)
        #del Z_function_u_; del Z_function_D_; del mu_matrix_u_; del std_matrix_u_;
        #del mu_matrix_D_; del std_matrix_D_
        
            # mu is the average of the source from which samples are generated, but the real normalization for
            # the action applied and used in the cost function is made respect to the previous average function
        
        return best_action_n_seq_, best_damping_n_seq_  # normalized quantity

    # Input data are normalized
    def Actor_train(self, state_force_train, Cost_norm, Actor_NN, Critic_NN, Model_NN, xy_norm, Nh, learning_rate):
        
        Actor_NN_ = Actor_NN
        Critic_NN_ = Critic_NN
        Model_NN_ = Model_NN
        Nh_ = Nh
        learning_rate_ = learning_rate
        xy_norm_ = xy_norm
        
        # variables to normalize
        x_mean_v_, x_std_v_, y_mean_v_, y_std_v_ = xy_norm_
        # denormalization
        state_force_train_not_norm_ = state_force_train*x_std_v_[0:3] + x_mean_v_[0:3]
        
        # normalized quantities
        state_force_trainp1_ = np.copy(state_force_train[1:Nh_+1, :])
        
        Actor_NN_.to("cpu")  # putting the model into GPU (but not available now)
        #optimizer_A = torch.optim.Adam(Actor_NN_.layers["lin" + str(2)].parameters(), lr =learning_rate_)
        optimizer_A = torch.optim.Adam(Actor_NN_.get_parameters(), lr =learning_rate_)
        errore_f = nn.MSELoss()
    
        actor_input_torch_ = torch.tensor(state_force_trainp1_, dtype=torch.float32, device="cpu")
        
        for j in range(0, Nh_):
            
            Actor_NN_.train()
            action_damping_train = Actor_NN_.forward(actor_input_torch_[j,:])
            optimizer_A.zero_grad()
            
            # Cross entropy method to estimate u minimizing the output of the critic network
            print("Input NN", state_force_trainp1_[j,:]*x_std_v_[0:3] + x_mean_v_[0:3])
            print("Input CEM",state_force_train_not_norm_[j+1:j+2])
            U_npiu1, D_npiu1 = self.CEM_critic(state_force_train_not_norm_[j+1:j+2], 1, 5, 64, xy_norm_, Cost_norm, Model_NN_, 
                                 Critic_NN_, num_ensembles_cem_= 5)
            U_D_npiu1 = torch.Tensor([U_npiu1[0,0],D_npiu1[0,0]])
            #U_npiu1_not_norm = CEM_norm_p(state_force_train_not_norm_[j+1:j+2], 1, 5, 64, xy_norm,
            #                              Model_NN_, num_ensembles_cem_= 5)[0]
            #U_npiu1 = (U_npiu1_not_norm - x_mean_v_[3:4])/x_std_v_[3:4]
    
            action_damping_from_NN = action_damping_train
            action_damping_from_CEM = U_D_npiu1
            print("NN ", action_damping_from_NN)
            print("CEM ", action_damping_from_CEM)
            # Cost function
            Error_a = errore_f.forward(action_damping_from_NN, action_damping_from_CEM)
            Error_a.backward(retain_graph=True)                           # backprop
            optimizer_A.step()                             # updating the parameters
        
        return
    
    def measurements_callback(self, pose, velocity, wrench):
        # the three inputs should represent three messages
        
        if (self.iter_buffer_ < self.prediction_horizon):
            # print("Inside Callback")
            EXTERNAL_FORCES   = np.array([wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z])
            print("msg.Wrench: ", EXTERNAL_FORCES)
            
            CARTESIAN_POSE = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y,
                                       pose.pose.orientation.z, pose.pose.orientation.w])
            CARTESIAN_VEL = np.array([velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z, velocity.twist.angular.x, velocity.twist.angular.y,
                                      velocity.twist.angular.z]) 
            
            z_norm = np.array([ (CARTESIAN_VEL[2]-self.x_mean_v[0])/self.x_std_v[0] ])
            x_norm = np.array([ (CARTESIAN_POSE[2]-self.x_mean_v[1])/self.x_std_v[1] ])
            fh_norm = np.array([ (EXTERNAL_FORCES[2]-self.x_mean_v[2])/self.x_std_v[2] ])
            actor_input_norm = np.array([z_norm, x_norm, fh_norm]).T
            
            self.actor_NN.to("cpu")
            self.actor_NN.eval()
            # normalization of the input of the actor network
            actor_data_torch_ = torch.tensor(actor_input_norm, dtype=torch.float32, device="cpu")
            u_D_3_norm = self.actor_NN.forward(actor_data_torch_)
            # denormalization
            self.u_3 = (u_D_3_norm[0,0]*self.x_std_v[3] + self.x_mean_v[3]).detach().numpy()
            self.Dr_3 = (u_D_3_norm[0,1]*self.x_std_v[4] + self.x_mean_v[4]).detach().numpy()
            print("Action generated", self.u_3)
            print("Damping generated", self.Dr_3)
            print(u_D_3_norm[0,0])
            print(self.u_record_norm[self.iter_buffer_])
            self.u_record_norm[self.iter_buffer_,0] = u_D_3_norm[0,0]
            self.D_record_norm[self.iter_buffer_,0] = u_D_3_norm[0,1]
            self.z_record_norm[self.iter_buffer_,0] = z_norm
            self.x_record_norm[self.iter_buffer_,0] = x_norm
            self.fh_record_norm[self.iter_buffer_,0] = fh_norm
            
            u_message = geometry_msgs.msg.PoseStamped()
            
            u_message.pose.position.x = pose.pose.position.x
            u_message.pose.position.y = pose.pose.position.y                   
            u_message.pose.position.z = self.u_3
            u_message.pose.orientation.x = pose.pose.orientation.x                           
            u_message.pose.orientation.y = pose.pose.orientation.y            
            u_message.pose.orientation.z = pose.pose.orientation.z  
            u_message.pose.orientation.w = pose.pose.orientation.w  
            
            self.u_pub.publish(u_message)
            
            D_message = std_msgs.msg.Float64
            
            D_message = self.Dr_3
            
            self.D_pub.publish(D_message)    
                
            self.iter_buffer_ += 1
        
        if (self.iter_buffer_ % self.prediction_horizon == 0):
            
            EXTERNAL_FORCES   = np.array([wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z])
            CARTESIAN_POSE = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y,
                                   pose.pose.orientation.z, pose.pose.orientation.w])
            CARTESIAN_VEL = np.array([velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z, velocity.twist.angular.x, velocity.twist.angular.y,
                                  velocity.twist.angular.z]) 
            z_norm = np.array([ (CARTESIAN_VEL[2]-self.x_mean_v[0])/self.x_std_v[0] ])
            x_norm = np.array([ (CARTESIAN_POSE[2]-self.x_mean_v[1])/self.x_std_v[1] ])
            fh_norm = np.array([ (EXTERNAL_FORCES[2]-self.x_mean_v[2])/self.x_std_v[2] ])
            self.z_record_norm[self.iter_buffer_,0] = z_norm
            self.x_record_norm[self.iter_buffer_,0] = x_norm
            self.fh_record_norm[self.iter_buffer_,0] = fh_norm
                
            z_tra = self.z_record_norm
            x_tra = self.x_record_norm
            fh_record_tra = self.fh_record_norm
            state_norm_tra = np.append(z_tra, x_tra, axis=1)
            state_force_norm_tra = np.append(state_norm_tra, fh_record_tra, axis=1)
            u_record_tra = self.u_record_norm
            D_record_tra = self.D_record_norm
            print("x: ", self.x_record_norm)
            
            for n_rete in range(self.ensemble_size):
                self.model_approximator_train(state_force_norm_tra, u_record_tra, D_record_tra, self.NN_ensemble["NN" + str(n_rete)], self.prediction_horizon,
                                              learning_rate = 1e-3)
            
    
            print("CRITIC")
            self.Critic_train(state_force_norm_tra, u_record_tra, D_record_tra, self.actor_NN, self.critic_NN, self.training_dict['xy_norm'], self.Cost_norm, self.prediction_horizon, 
                         learning_rate = 5e-5)
            print("ACTOR")
            self.Actor_train(state_force_norm_tra, self.Cost_norm, self.actor_NN, self.critic_NN, self.NN_ensemble, self.training_dict['xy_norm'], self.prediction_horizon, learning_rate = 5e-3)
            
            self.z_record_norm = np.zeros((self.prediction_horizon + 1, 1))
            self.x_record_norm = np.zeros((self.prediction_horizon + 1, 1))
            self.fh_record_norm = np.zeros((self.prediction_horizon + 1, 1))
            self.u_record_norm = np.zeros((self.prediction_horizon, 1))
            self.D_record_norm = np.zeros((self.prediction_horizon, 1))
            self.iter_buffer_ = 0
            
            
        self.rate.sleep()
        
    
# Main function.

    
if __name__=='__main__':
    
    print("Main function is started")
    # Node initialization
    rospy.init_node('Q_LMPC', anonymous=True)
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("Using {} device".format(device))
    #torch.cuda.device(0)
    #print(torch.cuda.get_device_name(0))
    #print(torch.cuda.get_device_properties(0))

    # Loading the Data
    a_file = open("data.pkl", "rb")
    training_dict = pickle.load(a_file)
    #training_dict = torch.jit.load('tensors.pt')
    x_mean_v, x_std_v, y_mean_v, y_std_v = training_dict['xy_norm']
    xn_train = training_dict['xn_train']
    yn_train = training_dict['yn_train']

    # Required data for NN models
    n_x = xn_train.shape[1]                        # input_size
    n_y = yn_train.shape[1]                        # output_size
    n_d = 5                                        # depth of the hidden layers
    n_h = 512                                      # size of the hidden layers
    num_ensembles = 5                              # number of NN in the ensemble
    T=5                                            # prediction horizon
    N=64                                           # number of samples
    print_loss = True
    dx = yn_train.shape[1]
    model_approximator = OrderedDict()

    # Initializing the NN models
    for i in range(num_ensembles):
        model_approximator["NN" + str(i)] = NN_model(n_x, n_d, n_h, n_y, print_NN=False)    

    for i in range(num_ensembles):
        model_approximator["NN" + str(i)].to("cpu")
        model_approximator["NN" + str(i)].eval()
    
    actor = ActorNN().to(device)
    critic = CriticNN().to(device)
    print("QLMPC controller is starting")
    myController = Q_LMPC(model_approximator, actor, critic, num_ensembles, T, N)
    rospy.spin()

