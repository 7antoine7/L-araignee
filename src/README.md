## Robot_logic.cpp
c'est la node qui controle l'araignée. Elle recoit les commandes de la manette par le topic "joy" et envoie les positions que les servomoteurs doivent prendre sur le topic serial_topic pour qu'ils soient envoyés en série au SSC32U. La node robot_logic est composée de deux machines a états, un qui gère les états de l'araignée et l'autre qui gère les changements d'état.
## Talker_serie.cpp
c'est la node qui communique avec le SSC32U. Elle recoit les positions des servo par une matrice sur le topic serial_topic et transforme les positions en chaine de caractères qui commandes avec le SSC32U.
