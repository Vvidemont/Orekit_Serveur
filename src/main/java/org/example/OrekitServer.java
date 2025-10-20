package org.example;
// Déclare le package du projet (doit correspondre à la structure du dossier src/main/java).

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.web.bind.annotation.RestController;
// Importation des classes Spring nécessaires au serveur REST.

@SpringBootApplication
// Indique que c’est une application Spring Boot (auto-configuration, scan des composants, etc.).
@RestController
// Déclare cette classe comme contrôleur REST principal (peut exposer des endpoints HTTP).

public class OrekitServer {
    // Classe principale du serveur Orekit (point d’entrée de l’application).

    public static void main(String[] args) {
        // Méthode exécutée au démarrage. Lance le serveur Spring Boot.
        SpringApplication.run(OrekitServer.class, args);
    }
}
