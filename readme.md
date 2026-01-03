Pour initialiser le projet : 

Doit avoir esp-idf d'installer comme extension dans vs code

git clone https://github.com/olivier-hamel/autonomous-robot.git

cd autonomous-robot

git submodule update --init --recursive

Sourcer esp-idf : source /chemin/vers/esp-idf/export.sh
Le chemin depend de votre installation

Ou ouvrir le projet dans vscode, faire ctrl + shfit + p et selectionner l'option reconfigure

idf.py set-target esp32c6

idf.py reconfigure

idf.py build

idf.py flash
