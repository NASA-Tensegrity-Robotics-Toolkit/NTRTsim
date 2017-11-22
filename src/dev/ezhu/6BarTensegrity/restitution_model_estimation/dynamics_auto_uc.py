import tensorflow as tf
import numpy as np
import math

# Predefined function to build a feedforward neural network
def build_mlp(input_placeholder,
              output_size,
              scope,
              n_layers=2,
              size=500,
              activation=tf.tanh,
              output_activation=None
              ):
    out = input_placeholder
    with tf.variable_scope(scope):
        for i in range(n_layers):
            out = tf.layers.dense(out, size, activation=activation, name='layer_'+str(i))
        out = tf.layers.dense(out, output_size, activation=output_activation, name='layer_out')
    return out

def shuffle(a, o):
    indicies = np.arange(o.shape[0])
    np.random.shuffle(indicies)

    a_shuf = a[indicies,:]
    o_shuf = o[indicies,:]

    return a_shuf, o_shuf

class NNDynamicsModel_Auto_UC():
    def __init__(self,
                 in_dim,
                 out_dim,
                 n_layers,
                 size,
                 activation,
                 output_activation,
                 normalization,
                 batch_size,
                 iterations,
                 learning_rate,
                 sess
                 ):
        """ Note: Be careful about normalization """
        print('Initializing dynamics model...')

        # Define other parameters
        self.print_int = 100
        self.eps = 1e-6
        self.n_layers = n_layers

        self.normalization = normalization
        self.batch_size = batch_size
        self.iterations = iterations
        self.sess = sess

        # Create mlp, loss, and training op
        self.norm_obs_ph = tf.placeholder(tf.float32,[None,in_dim])
        self.norm_deltas_act = tf.placeholder(tf.float32,[None,out_dim])
        self.norm_deltas_pred = build_mlp(self.norm_obs_ph,out_dim,'dyn_model',n_layers,size,activation,output_activation)

        # Maximize log probability
        self.logstd = tf.Variable(0*tf.ones([out_dim]),dtype=tf.float32)
        self.logprob = -tf.reduce_sum(tf.divide((self.norm_deltas_act-self.norm_deltas_pred)**2,2*((tf.exp(self.logstd)+self.eps)**2)),1)-tf.reduce_sum(self.logstd)-tf.log(2*math.pi)
        self.loss = -tf.reduce_mean(self.logprob)

        # Minimize mean squared error
        # self.loss = tf.losses.mean_squared_error(self.norm_deltas_pred,self.norm_deltas_act,scope='mse_loss')

        self.train_op = tf.train.AdamOptimizer(learning_rate).minimize(self.loss)

        # Create sampling op
        self.norm_deltas_samp = self.norm_deltas_pred+tf.multiply(tf.exp(self.logstd),tf.random_normal([tf.shape(self.norm_deltas_pred)[0],out_dim]))

    def fit(self, data):
        """
        Write a function to take in a dataset of (unnormalized)states, (unnormalized)actions, (unnormalized)next_states and fit the dynamics model going from normalized states, normalized actions to normalized state differences (s_t+1 - s_t)
        """
        # Extract data
        obs_t = data['obs_t']
        obs_tp1 = data['obs_tp1']
        deltas = obs_tp1-obs_t

        # Normalize data
        norm_obs_t = np.divide((obs_t-self.normalization['mean_obs']),(self.normalization['std_obs']+self.eps))
        norm_deltas = np.divide((deltas-self.normalization['mean_deltas']),(self.normalization['std_deltas']+self.eps))

        # Shuffle data
        norm_obs_t, norm_deltas = shuffle(norm_obs_t,norm_deltas)

        batch_counter = 0
        n_data = norm_obs_t.shape[0]
        n_batches = int(n_data/self.batch_size)

        self.sess.run(tf.global_variables_initializer())

        # Do learning, self.iterations is the number of epochs
        for i in range(self.iterations*(n_batches+1)):
            # Get batchs for inputs and labels
            start_pos = batch_counter*self.batch_size
            if batch_counter <= (n_batches-1):
                end_pos = (batch_counter+1)*self.batch_size
                batch_counter += 1
            else:
                end_pos = -1
                batch_counter = 0
            norm_obs_t_batch = norm_obs_t[start_pos:end_pos,:]
            norm_deltas_batch = norm_deltas[start_pos:end_pos,:]

            # Run training op
            self.sess.run(self.train_op,
                feed_dict={self.norm_obs_ph:norm_obs_t_batch,
                            self.norm_deltas_act:norm_deltas_batch})

            # Evaluate loss and print
            if i % self.print_int == 0 or i == self.iterations*(n_batches+1)-1:
                curr_loss = self.loss.eval(session=self.sess,
                    feed_dict={self.norm_obs_ph:norm_obs_t_batch,
                                self.norm_deltas_act:norm_deltas_batch})
                print('Iteration: %d, Loss: %g' % (i,curr_loss))

        print('Saving nn weights')
        for i in range(self.n_layers+1):
            model_name = 'dyn_model/'
            if i == self.n_layers:
                layer_name = 'layer_out'
                kernel = tf.get_default_graph().get_tensor_by_name(model_name+layer_name+'/kernel:0').eval(session=self.sess)
                bias = tf.get_default_graph().get_tensor_by_name(model_name+layer_name+'/bias:0').eval(session=self.sess)
                weights = np.vstack((kernel,bias.reshape(1,len(bias))))
                print(weights.shape)
            else:
                layer_name = 'layer_'+str(i)
                kernel = tf.get_default_graph().get_tensor_by_name(model_name+layer_name+'/kernel:0').eval(session=self.sess)
                bias = tf.get_default_graph().get_tensor_by_name(model_name+layer_name+'/bias:0').eval(session=self.sess)
                weights = np.vstack((kernel,bias.reshape(1,len(bias))))
                print(weights.shape)
            filename = './data/dynamics_nn_'+layer_name+'.csv'
            np.savetxt(filename,weights,delimiter=',')

        # Save normalization
        np.savetxt('./data/norm_obs.csv',np.vstack((self.normalization['mean_obs'],self.normalization['std_obs'])),delimiter=',')

        # Save standard deviations
        std = np.exp(self.logstd.eval(session=self.sess))
        np.savetxt('./data/std_vec.csv', std, delimiter=',')

    def predict(self, states):
        """ Write a function to take in a batch of (unnormalized) states and (unnormalized) actions and return the (unnormalized) next states as predicted by using the model """
        # First normalize
        norm_obs = np.divide((states-self.normalization['mean_obs']),(self.normalization['std_obs']+self.eps))

        # Use NN to predict normalized state change
        norm_deltas_mean = self.sess.run(self.norm_deltas_pred,feed_dict={self.norm_obs_ph:norm_obs})
        norm_deltas_samp = self.sess.run(self.norm_deltas_samp,feed_dict={self.norm_obs_ph:norm_obs})

        # Denormalize and get state prediction
        deltas_mean = self.normalization['mean_deltas']+np.multiply(self.normalization['std_deltas'],norm_deltas_mean)
        deltas_samp = self.normalization['mean_deltas']+np.multiply(self.normalization['std_deltas'],norm_deltas_samp)

        next_states_mean = states + deltas_mean
        next_states_samp = states + deltas_samp

        std = np.exp(self.logstd.eval(session=self.sess))

        return next_states_samp, next_states_mean, std
