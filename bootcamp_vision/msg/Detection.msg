# Une detection d'objet (TP03 YOLO -> back-projection sur la table).
#
# Type de REFERENCE fourni par bootcamp_vision. Dans ton propre projet, tu
# recreeras un equivalent dans TON paquet d'interfaces (placeholder des TP :
# mon_projet_interfaces) ; voir l'encart du README.

string  class_name      # nom de classe (ex. "coffee_pack_1")
int32   class_id        # identifiant de classe (rang YOLO)
float32 score           # confiance de la detection [0..1]

# --- Boite englobante 2D (repere image, en pixels) ---
float32 u               # centre x de la boite (px)
float32 v               # centre y de la boite (px)
float32 w               # largeur de la boite (px)
float32 h               # hauteur de la boite (px)

# --- Position sur la table (back-projection plan z=0.40), optionnel ---
# Rempli si on retro-projette (u,v) sur le plateau (cf. README, h=1.00 m).
# Laisser a zero si non calcule.
geometry_msgs/Point position    # X, Y, Z en metres (Z = 0.40 sur le plateau)
