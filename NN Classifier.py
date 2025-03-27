import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout
from tensorflow.keras.optimizers.legacy import Adam

# Hyperparameters
epochs_to_train = args.epochs or 60
learning_rate_value = args.learning_rate or 0.0005
ensure_determinism_flag = args.ensure_determinism
batch_size = args.batch_size or 32

# Shuffle data if non-determinism is not ensured
if not ensure_determinism_flag:
    train_dataset = train_dataset.shuffle(buffer_size=batch_size * 4)
train_dataset = train_dataset.batch(batch_size, drop_remainder=False)
validation_dataset = validation_dataset.batch(batch_size, drop_remainder=False)

# Model architecture using Sequential API
model = Sequential([
    Dense(20, activation='relu', activity_regularizer=tf.keras.regularizers.l1(0.00001)),
    Dropout(0.2),  # Added Dropout layer with rate 0.2
    Dense(10, activation='relu', activity_regularizer=tf.keras.regularizers.l1(0.00001)),
    Dense(classes, name='y_pred', activation='softmax')
])

# Optimizer setup
optimizer = Adam(learning_rate=learning_rate_value, beta_1=0.9, beta_2=0.999)

# Add custom callback (assuming BatchLoggerCallback is defined elsewhere)
callbacks.append(BatchLoggerCallback(batch_size, train_sample_count, epochs=epochs_to_train, ensure_determinism=ensure_determinism_flag))

# Compile model
model.compile(loss='categorical_crossentropy', optimizer=optimizer, metrics=['accuracy'])

# Train the model
model.fit(train_dataset, epochs=epochs_to_train, validation_data=validation_dataset, verbose=2, callbacks=callbacks)

# Option to disable per-channel quantization if necessary
disable_per_channel_quantization = False
