# Modeli kaydetme
import joblib
# Kaydedilen modeli yükleme
loaded_model = joblib.load("/home/cenk/regression_model2.joblib")

# Gerçek zamanlı kullanım
distance = 0.366
bbox_x = 205
bbox_y = 170
features = [[distance, bbox_x, bbox_y]]
radius = loaded_model.predict(features)
print("Tahmin edilen yarıçap:", radius)
