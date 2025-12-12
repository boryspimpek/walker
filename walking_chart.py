import numpy as np
import matplotlib.pyplot as plt

# ===== PARAMETRY =====
A = 0      # Poziom bazowy
B = 1       # Amplituda
C = 3      # Stromość przejść
D = 4       # Okres funkcji
F = 0.175     # Przesunięcie fazowe (w ułamku okresu)
swing_time = 0.49
# =====================

def f_elegant(x, a=A, b=B, c=C, d=D, f=F):
    t = (x - f*d) % d
    t_normalized = 2 * t / d
    triangle = 1 - 2 * abs((t_normalized % 2) - 1)
    return a + b * np.tanh(c * triangle) / np.tanh(c)

def f_cosinus(x, a=A, b=B, d=D, f=F):
    t = (x - f*d) % d
    return a + b * np.cos(np.pi * t / (d/2))

def f_kwadratowy(x, a=A, b=B, d=D):
    t = x % d
    if t < swing_time * d:
        return a + b
    elif t < 0.5 * d:
        return a
    elif t < (0.5 + swing_time) * d:
        return a - b
    else:
        return a

# Generowanie danych
x = np.linspace(0, 4, 1000)
y1 = f_elegant(x)
y2 = -f_cosinus(x)
y3 = np.vectorize(f_kwadratowy)(x)

# Wykres
plt.figure(figsize=(10, 4))
plt.plot(x, y1, 'b-', linewidth=2, label=f'Funkcja elegancka (F={F})')
plt.plot(x, y2, 'g-', linewidth=1.5, alpha=0.7, label=f'Cosinus (F={F})')
plt.plot(x, y3, 'r-', linewidth=2, alpha=0.7, label='Funkcja kwadratowa')
plt.title(f'Porównanie funkcji z przesunięciem fazowym F={F}')
plt.xlabel('x')
plt.ylabel('y')
plt.grid(True, alpha=0.3)
plt.legend()
plt.show()