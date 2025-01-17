import math

def newton_raphson(a, x, y0, tol = 1e-6, max_iter=100):
    def f(y):
        return math.sin(y/a) -y/x
    def f_prime(y):
        return (1/a)*math.cos(y/a) -1/x
    
    y = y0
    for i in range(max_iter):
        f_y = f(y)
        f_prime_y = f_prime(y)

        if abs(f_prime_y) < 1e-12:
            raise ValueError("Dérivée trop proche de 0")
        
        y_new = y - f_y /f_prime_y

        if abs(y_new - y) < tol:
            return y_new
        
        y= y_new
    
    raise ValueError("Pas de convergence")