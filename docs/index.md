
## CodinGame

### Site URL

<https://www.codingame.com/training/easy>

### Google docs

<https://docs.google.com/spreadsheets/d/1koBeSrQ3yDxVRiUkARXx9e9FwEVwaJktRdL4o3opCLo/edit?usp=sharing>


<br/>

---

## Python

### Liens

#### Cours en ligne sur Open Class Rooms :

<https://openclassrooms.com/fr/courses/235344-apprenez-a-programmer-en-python>

N'utiliser que les cours gratuits sans vidéo.

#### Doc Python 3 :

<https://docs.python.org/fr/3.6/>

<br/>

### Les commentaires

```py
# Un commentaire
```

<br/>

### Les variables

```py
a = 2
type(a)
# ⇨ <class 'int'> ⇨ un entier

c = 3.14
type(c)
# ⇨ <class 'float'> ⇨ un réel

b = "coucou !"
type(b)
# ⇨ <class 'str'> ⇨ une chaine de caractères

a = a + 7 # ⇨ a contient maintenant la valeur 9

a = a + b # ⇨ Exception : TypeError:
          #               unsupported operand type(s) for +: 'int' and 'str'

une_variableAvecUnNom_BIZARRE = "toto"
```

<br/>

### Les conditions

```py
a = 56
if a == 56:
    print("c'est vrai")

if a > 10:
    print("c'est vrai")
elif a > 40:
    print("c'est vrai mais on ne passera pas ici")
else:
    print("on ne passera pas ici")


if a == 56 and True:
    print("c'est vrai")

if a == 56 or False:
    print("c'est vrai")
```

<br/>

### Les boucles

```py
i = 0
while i < 10:
    print(i) # ⇨ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    i = i + 1


for i in range(10):
    print(i) # ⇨ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9


tableau = ['abcd', 'toto', 42, 90, '90']
for i in tableau:
    print(i) # ⇨ abcd, toto, 42, 90, 90


string = 'hello'
for letter in string:
    print(letter) # ⇨ h, e, l, l, o
```

<br/>

### Les fonctions
```py
print("coucou") # ⇨ c'est la fonction print
type("coucou") # ⇨ c'est la fonction type

def une_fonction(a, b, c = 12):
    # Ici on peut mettre plein de code
    variable_intermediaire = (a + b) / c

    # on retourne un résultat
    return variable_intermediaire


a = une_fonction(6, 18) # ⇨ 2.0
b = une_fonction(2, 6, 2) # ⇨ 4.0

print(a - b) # ⇨ - 2.0
```

<br/>


### Les try-catch (ou try-except)
```py
try:
    print("une instruction ou tout va bien")
except:
    print("on ne passe pas ici")

try:
    12 / 0
    print("on ne passe pas ici") # ⇨  après l'exception
except ZeroDivisionError as a:
    print("on passe ici")
    print(a)
```

<br/>

### Les classes (programmation objet)

```py
class Vehicule:
    def __init__(self, type): # fonction de construction de l'objet
        self.nb_roues = 'inconnu' # variable nb_roues créée dans à l'objet
        if type == 'voiture':
            self.nb_roues = 4

    def nombre_de_roues(self):
        return self.nb_roues

    def ajouter_une_roue(self):
        self.nb_roues = self.nb_roues + 1


mon_vehicule = Vehicule('voiture')
print( mon_vehicule.nombre_de_roues() ) # ⇨ 4
mon_vehicule.ajouter_une_roue()
print( mon_vehicule.nombre_de_roues() ) # ⇨ 5
```

<br/>

---

## TP seed

```py
# -*- coding: utf-8 -*-

# 1. Launch windows powershell
# 2. Go where you saved the file, for example:
#      > cd c:\Users\gim\Desktop
# 3. Use python 2.7 (keyboard issue with python 3.7):
#      > C:\Python27\python.exe '.\file.py'

import keyboard
import requests

while True:
    key = keyboard.read_event()
    if key.event_type == 'up':
        continue

    print key.name
    print key.scan_code
```
