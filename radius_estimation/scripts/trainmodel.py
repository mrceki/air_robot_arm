# Gerekli kütüphaneleri yükleme
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.linear_model import LinearRegression, Ridge, Lasso, ElasticNet

# Verileri yükleme
data = pd.read_csv("combined.csv", header=None, names=["radius", "depth", "bbox_x", "bbox_y"])

# Verileri özellik ve hedef değişkenleri olarak ayırma
X = data.drop("radius", axis=1)
y = data["radius"]

# Train ve test verilerini ayırma
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Ridge Regresyon modelini eğitme ve optimum hiperparametreleri arama
ridge = Ridge()
params = {'alpha': [0.001, 0.01, 0.1, 1, 10]}
grid_ridge = GridSearchCV(estimator=ridge, param_grid=params, scoring='neg_mean_squared_error', cv=5)
grid_ridge.fit(X_train, y_train)
print("Ridge Regresyon Modeli En İyi Parametreler:", grid_ridge.best_params_)

# Lasso Regresyon modelini eğitme ve optimum hiperparametreleri arama
lasso = Lasso()
params = {'alpha': [0.001, 0.01, 0.1, 1, 10]}
grid_lasso = GridSearchCV(estimator=lasso, param_grid=params, scoring='neg_mean_squared_error', cv=5)
grid_lasso.fit(X_train, y_train)
print("Lasso Regresyon Modeli En İyi Parametreler:", grid_lasso.best_params_)

# Elastic Net Regresyon modelini eğitme ve optimum hiperparametreleri arama
elastic = ElasticNet()
params = {'alpha': [0.001, 0.01, 0.1, 1, 10], 'l1_ratio': [0.1, 0.3, 0.5, 0.7, 0.9]}
grid_elastic = GridSearchCV(estimator=elastic, param_grid=params, scoring='neg_mean_squared_error', cv=5)
grid_elastic.fit(X_train, y_train)
print("Elastic Net Regresyon Modeli En İyi Parametreler:", grid_elastic.best_params_)

# En iyi modeli seçme ve eğitme
best_model = None
best_score = -np.inf

for model in [LinearRegression(), grid_ridge.best_estimator_, grid_lasso.best_estimator_, grid_elastic.best_estimator_]:
    model.fit(X_train, y_train)
    score = model.score(X_test, y_test)
    print("Model:", model.__class__.__name__, "R^2 Score:", score)
    if score > best_score:
        best_score = score
        best_model = model

print(best_model)
# Modeli kaydetme
import joblib
joblib.dump(best_model, "regression_model.joblib")
