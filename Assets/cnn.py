import os
import numpy as np
from sklearn.model_selection import train_test_split
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.utils import to_categorical
import tensorflow as tf
import cv2
import matplotlib.pyplot as plt
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau

# Paths
data_dir = r"D:\imgs"  # Main directory containing class subdirectories
classes = ['bottle', 'can']       # Class labels

# Parameters
img_size = (224, 224)
test_size = 0.2  # 20% of data for testing
batch_size = 32

# Function to load and preprocess all images
def load_images(data_dir, classes, img_size):
    images = []
    labels = []
    for label, class_name in enumerate(classes):
        class_dir = os.path.join(data_dir, class_name)
        if not os.path.exists(class_dir):
            continue
        for file in os.listdir(class_dir):
            if file.endswith(('.png', '.jpg', '.jpeg')):
                img_path = os.path.join(class_dir, file)
                img = cv2.imread(img_path)
                if img is not None:
                    # Resize and normalize the image
                    img = cv2.resize(img, img_size) / 255.0
                    images.append(img)
                    labels.append(label)
    return np.array(images), np.array(labels)

# Load dataset
images, labels = load_images(data_dir, classes, img_size)

# Split dataset into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(images, labels, test_size=test_size, random_state=42)

# Convert labels to one-hot encoding
y_train = to_categorical(y_train, num_classes=len(classes))
y_test = to_categorical(y_test, num_classes=len(classes))

# Define data generators for augmentation
train_datagen = ImageDataGenerator(
    rotation_range=30,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True,
    fill_mode='nearest'
)

test_datagen = ImageDataGenerator()

train_generator = train_datagen.flow(
    X_train, y_train, batch_size=batch_size
)

test_generator = test_datagen.flow(
    X_test, y_test, batch_size=batch_size
)

# Load pre-trained base model with locally stored weights
base_model = MobileNetV2(
    input_shape=(224, 224, 3),
    include_top=False,
    weights=r"D:\Unity projects\CSE5369-GH\mobilenet_v2_weights_tf_dim_ordering_tf_kernels_1.0_224_no_top.h5"
)
base_model.trainable = False  # Freeze the base model weights

# Define the full model
model = tf.keras.Sequential([
    base_model,
    tf.keras.layers.GlobalAveragePooling2D(),
    tf.keras.layers.Dense(128, activation='relu', kernel_regularizer=tf.keras.regularizers.l2(0.01)),
    tf.keras.layers.Dropout(0.5),
    tf.keras.layers.Dense(len(classes), activation='softmax')
])

# Compile the model
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

# Callbacks for Early Stopping and Learning Rate Adjustment
early_stopping = EarlyStopping(monitor='val_loss', patience=5, restore_best_weights=True)
lr_scheduler = ReduceLROnPlateau(monitor='val_loss', factor=0.1, patience=3)

# Train the model and store history
history = model.fit(
    train_generator,
    epochs=20,
    validation_data=test_generator,
    callbacks=[early_stopping, lr_scheduler]
)

# Save the model
model.save("bottle_can_classifier.h5")
print("Model trained and saved as bottle_can_classifier.h5")

# Plot training and validation accuracy/loss
def plot_training_history(history):
    acc = history.history['accuracy']
    val_acc = history.history['val_accuracy']
    loss = history.history['loss']
    val_loss = history.history['val_loss']
    epochs = range(1, len(acc) + 1)

    plt.figure(figsize=(12, 6))

    # Plot accuracy
    plt.subplot(1, 2, 1)
    plt.plot(epochs, acc, 'b', label='Training Accuracy')
    plt.plot(epochs, val_acc, 'r', label='Validation Accuracy')
    plt.title('Training and Validation Accuracy')
    plt.xlabel('Epochs')
    plt.ylabel('Accuracy')
    plt.legend()

    # Plot loss
    plt.subplot(1, 2, 2)
    plt.plot(epochs, loss, 'b', label='Training Loss')
    plt.plot(epochs, val_loss, 'r', label='Validation Loss')
    plt.title('Training and Validation Loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()

    plt.tight_layout()
    plt.show()

plot_training_history(history)

# Evaluate the model on test data
test_loss, test_accuracy = model.evaluate(test_generator)
print(f"Test Accuracy: {test_accuracy * 100:.2f}%")

# Predict on a few test images
def display_predictions(X_test, y_test, model, classes, num_samples=5):
    indices = np.random.choice(len(X_test), num_samples, replace=False)
    for i in indices:
        img = X_test[i]
        label = np.argmax(y_test[i])
        pred = np.argmax(model.predict(np.expand_dims(img, axis=0)))
        
        plt.imshow(img)
        plt.title(f"True: {classes[label]}, Predicted: {classes[pred]}")
        plt.axis('off')
        plt.show()
