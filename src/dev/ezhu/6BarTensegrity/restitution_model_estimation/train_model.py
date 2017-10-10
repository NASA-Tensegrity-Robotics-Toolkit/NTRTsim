import tensorflow as tf
import numpy as np
import math
import time
import preprocessing

def build_mlp(
        input_placeholder,
        output_size,
        scope,
        n_layers=2,
        size=64,
        activation=tf.tanh,
        output_activation=None):

    with tf.name_scope(scope):
        for layer in range(n_layers+1):
            # Input layer
            if layer == 0:
                out = tf.layers.dense(input_placeholder,size,activation,
                    # kernel_initializer=tf.truncated_normal_initializer,
                    kernel_initializer=tf.contrib.layers.xavier_initializer(),
                    bias_initializer=tf.constant_initializer(0.1),
                    kernel_regularizer=tf.nn.l2_loss)
            # Output layer
            elif layer == n_layers:
                out = tf.layers.dense(out,output_size,output_activation,
                    # kernel_initializer=tf.truncated_normal_initializer,
                    kernel_initializer=tf.contrib.layers.xavier_initializer(),
                    bias_initializer=tf.constant_initializer(0.1),
                    kernel_regularizer=tf.nn.l2_loss)
            else:
                out = tf.layers.dense(out,size,activation,
                    # kernel_initializer=tf.truncated_normal_initializer,
                    kernel_initializer=tf.contrib.layers.xavier_initializer(),
                    bias_initializer=tf.constant_initializer(0.1),
                    kernel_regularizer=tf.nn.l2_loss)
    return out

def train(args, x_train, y_train):
    SAVE_INT = 1000
    SAVE_STEP = 0
    PRINT_INT = 1000
    x_dim = x_train.shape[1]
    y_dim = y_train.shape[1]
    n_data = x_train.shape[0]

    # Define placeholders for input and actual output
    x_ph = tf.placeholder(tf.float32,[None,x_dim])
    y_ph = tf.placeholder(tf.float32,[None,y_dim])

    # Build nn and loss
    y_mean = build_mlp(x_ph,y_dim,'restitution_model',args.n_layers,args.size,tf.tanh,None)
    logstd = tf.Variable(0*tf.ones([y_dim]),dtype=tf.float32)
    # sampled_y = y_mean+tf.multiply(tf.exp(logstd),tf.random_normal([tf.shape(y_mean)[0],y_dim]))
    logprob = -tf.reduce_sum(tf.divide((y_ph-y_mean)**2,2*(tf.exp(logstd)**2)),1)-tf.reduce_sum(logstd)-tf.log(2*math.pi)

    loss = -tf.reduce_mean(logprog)
    update_op = tf.train.AdamOptimzer(args.learning_rate).minimize(loss)

    n_batches = int(n_data/args.batch_size)
    batch_counter = 0

    iters = []
    loss_val = []

    tf_config = tf.ConfigProto(inter_op_parallelism_threads=1, intra_op_parallelism_threads=1)
    sess = tf.Session(config=tf_config)
    sess.__enter__()
    sess.run(tf.global_variables_initializer())

    saver = tf.train.Saver()

    x_train, x_mean, x_std = preprocessing.standardize(x_train)
    x_mean_save = tf.constant(x_mean, name='x_mean')
    x_std_save = tf.constant(x_std, name='x_std')

    for i in range(n_iter):
        start_pos = batch_counter*args.batch_size
        if batch_counter <= (n_batches-1):
            if start_pos == 0:
                x_train, y_train = preprocessing.shuffle(x_train, y_train)
            end_pos = (batch_counter+1)*batch_size
            batch_counter += 1
        else:
            end_pos = -1
            batch_counter = 0

        x_batch = x_train[start_pos:end_pos,:]
        y_batch = y_train[start_pos:end_pos,:]

        sess.run(update_op,feed_dict={x_ph:x_batch,y_ph:y_batch})

    if i % PRINT_INT == 0 or i == n_iter-1:
        curr_loss = loss.eval(x_ph:x_batch,y_ph:y_batch)
        print('Iteration: %d, Loss: %g' % (i,curr_loss))
        iters.append(i)
        loss_val.append(curr_loss)

    if i % SAVE_INT == 0:
        print('Saving model at iteration %d' % (i))
        model_name = './models/restiution_model_'+time.strftime("%d-%m-%Y_%H-%M-%S")
        model_path = saver.save(sess,model_name,global_step=SAVE_STEP)
        SAVE_STEP += 1

def test(args, x_test, y_test):
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
	parser.add_argument('mode', type=str, default='train')
    parser.add_argument('n_layers', type=int, default=2)
    parser.add_argument('size', type=int, default=32)
    parser.add_argument('learning_rate', type=float, default=5e-3)
    parser.add_argument('n_iter', type=int, default=50000)
    parser.add_argument('batch_size', type=int, default=100)
    args = parser.parse_args()

    x_train, y_train, x_test, y_test = preprocessing.get_training_set()

    if args.mode == 'train':
        print('Train mode')
        train(args,x_train,y_train)
    elif args.mode = 'test':
        print('Test mode')
        test(args,x_test,y_test)
