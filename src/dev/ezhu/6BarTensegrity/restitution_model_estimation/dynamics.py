import tensorflow as tf
import numpy as np

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
        for _ in range(n_layers):
            out = tf.layers.dense(out, size, activation=activation)
        out = tf.layers.dense(out, output_size, activation=output_activation)
    return out

def shuffle(a, o):
    indicies = np.arange(o.shape[0])
    np.random.shuffle(indicies)

    a_shuf = a[indicies,:]
    o_shuf = o[indicies,:]

    return a_shuf, o_shuf

class NNDynamicsModel():
    def __init__(self,
                 env,
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
        obs_dim = env.observation_space.shape[0]
        ac_dim = env.action_space.shape[0]

        self.normalization = normalization
        self.batch_size = batch_size
        self.iterations = iterations
        self.sess = sess

        # Create mlp, loss, and training op
        self.norm_obs_ac_ph = tf.placeholder(tf.float32,[None,obs_dim+ac_dim])
        self.norm_deltas_act = tf.placeholder(tf.float32,[None,obs_dim])
        self.norm_deltas_pred = build_mlp(self.norm_obs_ac_ph,obs_dim,'dyn_model',n_layers,size,activation,output_activation)
        self.loss = tf.losses.mean_squared_error(self.norm_deltas_pred,self.norm_deltas_act,scope='mse_loss')
        self.train_op = tf.train.AdamOptimizer(learning_rate).minimize(self.loss)

        # Define other parameters
        self.print_int = 100
        self.eps = 1e-6

    def fit(self, data):
        """
        Write a function to take in a dataset of (unnormalized)states, (unnormalized)actions, (unnormalized)next_states and fit the dynamics model going from normalized states, normalized actions to normalized state differences (s_t+1 - s_t)
        """
        # Extract data
        obs_t = data['obs_t']
        obs_tp1 = data['obs_tp1']
        acs_t = data['acs_t']
        deltas = obs_tp1-obs_t

        # Normalize data
        norm_obs_t = np.divide((obs_t-self.normalization['mean_obs']),(self.normalization['std_obs']+self.eps))
        norm_acs_t = np.divide((acs_t-self.normalization['mean_acs']),(self.normalization['std_acs']+self.eps))
        norm_deltas = np.divide((deltas-self.normalization['mean_deltas']),(self.normalization['std_deltas']+self.eps))

        # Create matricies for supervised learning_rate
        norm_obs_acs = np.concatenate((norm_obs_t,norm_acs_t),axis=1)

        # Shuffle data
        norm_obs_acs, norm_deltas = shuffle(norm_obs_acs,norm_deltas)

        batch_counter = 0
        n_data = norm_obs_acs.shape[0]
        n_batches = int(n_data/self.batch_size)
        # iters = []
        # loss_val = []

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
            norm_obs_acs_batch = norm_obs_acs[start_pos:end_pos,:]
            norm_deltas_batch = norm_deltas[start_pos:end_pos,:]

            # Run training op
            self.sess.run(self.train_op,
                feed_dict={self.norm_obs_ac_ph:norm_obs_acs_batch,
                            self.norm_deltas_act:norm_deltas_batch})

            # Evaluate loss and print
            if i % self.print_int == 0 or i == self.iterations*(n_batches+1)-1:
                curr_loss = self.loss.eval(session=self.sess,
                    feed_dict={self.norm_obs_ac_ph:norm_obs_acs_batch,
                                self.norm_deltas_act:norm_deltas_batch})
                print('Iteration: %d, Loss: %g' % (i,curr_loss))

    def predict(self, states, actions):
        """ Write a function to take in a batch of (unnormalized) states and (unnormalized) actions and return the (unnormalized) next states as predicted by using the model """
        # First normalize
        norm_states = np.divide((states-self.normalization['mean_obs']),(self.normalization['std_obs']+self.eps))
        norm_actions = np.divide((actions-self.normalization['mean_acs']),(self.normalization['std_acs']+self.eps))
        if len(states.shape) == 1:
            norm_obs_ac = np.append(norm_states,norm_actions).reshape((1,len(states)+len(actions)))
        else:
            norm_obs_ac = np.concatenate((norm_states,norm_actions),axis=1)

        # Use NN to predict normalized state change
        norm_deltas = self.sess.run(self.norm_deltas_pred,feed_dict={self.norm_obs_ac_ph:norm_obs_ac})

        # Denormalize and get state prediction
        deltas = self.normalization['mean_deltas']+np.multiply(self.normalization['std_deltas'],norm_deltas)
        next_states = states + deltas

        return next_states
