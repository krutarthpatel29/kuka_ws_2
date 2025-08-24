# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.metrics import classification_report, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
import joblib
import os

# ==== Paths ====
output_dir = "/home/krutarth/Desktop/kuka_ws_2/scripts"
os.makedirs(output_dir, exist_ok=True)
data_file = os.path.join(output_dir, "kr360_ai_log_realistic.csv")
model_file = os.path.join(output_dir, "kr360_realistic_model.pkl")
report_file = os.path.join(output_dir, "kr360_realistic_report.txt")
conf_matrix_file = os.path.join(output_dir, "confusion_matrix_realistic.png")
feature_plot_file = os.path.join(output_dir, "feature_importance_realistic.png")

# ==== Load Data ====
df = pd.read_csv(data_file)
X = df.drop(columns=["timestamp", "label"])
y = df["label"]

# ==== Split Data ====
X_train, X_test, y_train, y_test = train_test_split(
    X, y, stratify=y, test_size=0.2, random_state=42
)

# ==== Train Model ====
model = RandomForestClassifier(n_estimators=100, random_state=42)
model.fit(X_train, y_train)

# ==== Evaluate ====
y_pred = model.predict(X_test)
report = classification_report(y_test, y_pred, digits=3)
print("\nClassification Report:\n", report)

with open(report_file, "w") as f:
    f.write("KR360 Anomaly Detection Report (Realistic Data)\n\n")
    f.write(report)

# ==== Confusion Matrix ====
conf_matrix = confusion_matrix(y_test, y_pred)
plt.figure(figsize=(5, 4))
sns.heatmap(conf_matrix, annot=True, fmt='d', cmap='Blues',
            xticklabels=["Normal", "Anomaly"],
            yticklabels=["Normal", "Anomaly"])
plt.title("Confusion Matrix")
plt.xlabel("Predicted")
plt.ylabel("Actual")
plt.tight_layout()
plt.savefig(conf_matrix_file)
plt.close()

# ==== Feature Importance ====
importances = model.feature_importances_
features = X.columns
plt.figure(figsize=(7, 4))
sns.barplot(x=importances, y=features)
plt.title("Feature Importance (Realistic Data)")
plt.tight_layout()
plt.savefig(feature_plot_file)
plt.close()

# ==== Cross-Validation ====
cv_scores = cross_val_score(model, X, y, cv=5)
print(f"5-Fold CV Accuracy: {cv_scores.mean():.3f} ± {cv_scores.std():.3f}")

# ==== Save Model ====
joblib.dump(model, model_file)
print(f"Model saved to: {model_file}")
print(f"Report: {report_file}")
print(f"Confusion Matrix: {conf_matrix_file}")
print(f"Feature Importance: {feature_plot_file}")

