# Matheuristique MPVRP-CC avec `CPSeqVar`

## 1. Portée du modèle

La classe [`MPVRPSeqVar`](../src/main/java/org/maxicp/cp/examples/raw/mpvrp/MPVRPSeqVar.java)
implémente une matheuristique pour le MPVRP-CC décrit dans
[`problem.md`](problem.md).

Ce modèle doit être distingué du modèle exact à variables successeurs de
[`MPVRP`](../src/main/java/org/maxicp/cp/examples/raw/mpvrp/MPVRP.java) :

- le prétraitement heuristique de `MPVRPSeqVar` fixe les quantités, les dépôts,
  la composition des voyages et l'ordre interne des stations ;
- le modèle CP affecte ensuite ces voyages aux véhicules et les ordonne ;
- l'optimalité éventuellement prouvée concerne donc les voyages construits, et
  non l'ensemble des solutions du MPVRP-CC original.

Toute solution écrite reste une solution réalisable du problème original. En
revanche, l'échec de la construction ou du modèle restreint ne constitue pas une
preuve d'infaisabilité de l'instance originale.

## 2. Décomposition en voyages atomiques

Le traitement comporte deux phases :

1. `buildTrips` fractionne les demandes, choisit leurs dépôts fournisseurs,
   regroupe les livraisons et fixe leur ordre interne ;
2. une `CPSeqVar` par véhicule affecte et ordonne les voyages construits.

Un voyage est un nœud atomique :

```text
Trip = DepotProduct + liste ordonnée de Request
```

Une `Request` contient :

```text
Request(logicalRequest, station, depot, quantity)
```

Pendant le prétraitement, elle est temporairement enveloppée dans :

```text
AllocatedDelivery(request, constructionVehicle)
```

`constructionVehicle` est l'indice réel du véhicule dont la capacité a servi à
créer la fraction. Cette information n'est pas écrite dans la solution finale et
ne fixe pas l'affectation CP ; elle sert uniquement à préserver un témoin de
réalisabilité pendant le regroupement.

Toutes les livraisons d'un voyage utilisent le même dépôt-produit. La quantité
chargée est exactement la somme des quantités livrées :

```text
trip.quantity() = somme(delivery.quantity())
```

La structure opérationnelle suivante est ainsi garantie par construction :

```text
Garage -> Depot -> Station -> ... -> Station -> Depot -> ... -> Garage
```

## 3. Construction des livraisons fractionnées

Les demandes sont traitées par quantité décroissante et les capacités des
véhicules par ordre décroissant. Une fraction est bornée par :

- la demande restante ;
- la capacité considérée ;
- le stock restant du dépôt-produit.

Une demande produit au plus une fraction par véhicule disponible. Cette limite
est cohérente avec la règle interdisant à un véhicule de servir deux fois la
même paire station-produit.

Deux stratégies d'allocation sont essayées :

1. `MAX_QUANTITY` sélectionne le dépôt fournissant la plus grande fraction et
   utilise la distance comme départage ;
2. si cette stratégie fragmente défavorablement les stocks,
   `PRESERVE_LARGE_STOCKS` recommence depuis les stocks initiaux et consomme
   d'abord les petits stocks, tout en réservant une quantité suffisante pour les
   capacités restantes.

Ce second passage corrige notamment le cas suivant :

```text
stocks des dépôts = [1, 1, 4]
demandes           = [3, 3]
véhicules           = deux véhicules de capacité 3
```

L'allocation gloutonne maximale consommerait trois unités du grand stock pour la
première demande et bloquerait la seconde. L'allocation de secours construit
deux répartitions `1 + 2`, qui sont réalisables.

Si les deux stratégies échouent, le programme indique explicitement que la
matheuristique n'a pas trouvé d'allocation. Il ne déclare pas l'instance
MPVRP-CC infaisable.

## 4. Regroupement des livraisons

Chaque fraction conserve le véhicule utilisé comme *témoin de construction* lors
du fractionnement. Les fractions sont ensuite triées par quantité décroissante
et regroupées par best-fit decreasing. Une fraction peut rejoindre un voyage
lorsque :

- elle utilise le même dépôt-produit ;
- elle possède le même véhicule témoin ;
- le voyage ne contient pas déjà la même demande logique ;
- la charge totale ne dépasse pas la capacité du véhicule témoin.

Cette affectation témoin n'est pas imposée au modèle CP : celui-ci peut réaffecter
un voyage à tout véhicule de capacité suffisante. Elle garantit toutefois qu'au
moins une affectation respecte simultanément les capacités et la règle des
véhicules distincts pour les fractions d'une même demande. Cela évite les
incohérences immédiates qui apparaissaient lorsque plusieurs voyages d'une même
demande devenaient tous compatibles avec un unique grand véhicule.

La construction continue d'exploiter les flottes hétérogènes : les fractions
associées à un grand véhicule peuvent être regroupées jusqu'à sa capacité, sans
forcer tous les voyages à respecter la capacité minimale de la flotte.

L'interdiction de regrouper deux fractions de la même demande est nécessaire :
ces fractions devront être affectées à des véhicules distincts.

### 4.1. Invariant de réalisabilité du regroupement

Pour chaque fraction `a` créée avec le véhicule témoin `k` :

```text
quantity(a) <= capacity(k)
```

Pour une demande logique donnée, l'allocation crée au plus une fraction avec un
même véhicule témoin. Comme un voyage ne regroupe que des fractions possédant le
même témoin et ne contient jamais deux fois la même demande logique, l'affectation
suivante est toujours disponible après la construction :

```text
chaque voyage construit avec le témoin k est affecté au véhicule k
```

Un véhicule peut exécuter plusieurs voyages ; cette affectation respecte donc :

- la capacité de chaque voyage ;
- l'affectation unique des voyages ;
- l'utilisation de véhicules distincts pour les fractions d'une même demande.

Le CP reste libre de trouver une meilleure affectation, mais ce témoin garantit
que les contraintes d'affectation ne sont pas contradictoires dès leur
publication.

### 4.2. Origine des anciennes incohérences

Une version intermédiaire regroupait toutes les fractions jusqu'à la capacité
maximale de la flotte, sans conserver leur véhicule de construction. Elle pouvait
alors produire plusieurs voyages d'une même demande qui n'étaient compatibles
qu'avec le même grand véhicule.

Par exemple, avec les capacités de l'instance 003 :

```text
[3181, 1753, 3698, 2895, 1808, 2958]
```

un voyage de charge `3698` ne peut être exécuté que par le véhicule d'indice 2.
Si plusieurs fractions d'une même demande étaient placées dans des voyages de ce
type, les contraintes imposaient simultanément :

```text
required[2, tripA] = 1
required[2, tripB] = 1
required[2, tripA] + required[2, tripB] <= 1
```

L'exception `InconsistencyException` apparaissait alors pendant la publication de
la contrainte de visite, avant le démarrage de la recherche. Les instances 003,
007 et 009 constituent des tests de non-régression pour ce cas.

## 5. Ordre interne des stations

Deux méthodes sont disponibles :

- `NEAREST_NEIGHBOR` construit un ordre par plus proche voisin ;
- `TWO_OPT`, méthode par défaut, améliore cet ordre par inversions successives.

Le score 2-opt contient la distance interne du voyage et la meilleure
continuation possible depuis sa dernière station vers un dépôt ou un garage :

```text
score(trip) = distance(depot -> stations ordonnées)
            + min distance(dernière station, prochain dépôt ou garage)
```

Cette continuation est une approximation indépendante de la route future. Elle
évite de choisir un chemin interne court qui se termine très loin de toute
destination plausible. L'ordre reste néanmoins fixé avant la recherche CP et
peut donc différer de l'ordre globalement optimal.

## 6. Variables de séquence

Soient `T` voyages et `K` véhicules. Le modèle crée `T + 2K` nœuds :

| Intervalle | Signification |
| --- | --- |
| `0 .. T-1` | voyages atomiques |
| `T .. T+K-1` | départs des véhicules |
| `T+K .. T+2K-1` | retours des véhicules |

Pour chaque véhicule `k` :

```java
routes[k] = makeSeqVar(cp, nodeCount, start[k], end[k]);
```

Les extrémités des autres véhicules sont exclues. La séquence directe
`start[k] -> end[k]` représente un véhicule inutilisé et coûte zéro.

## 7. Contraintes

### Affectation unique

Chaque voyage est requis dans exactement une route :

```text
somme(k, required[k, trip]) = 1
```

### Capacité

Lorsqu'un voyage dépasse la capacité d'un véhicule, il est exclu de sa route :

```text
trip.quantity() > capacity[k] => exclude(routes[k], trip)
```

### Visite station-produit

Pour chaque demande logique `r` et chaque véhicule `k` :

```text
somme(required[k, trip] pour les voyages contenant r) <= 1
```

Deux fractions de la même demande utilisent donc des véhicules distincts.

## 8. Coût entre voyages

Pour un voyage `t`, on note `d_t` son dépôt, `p_t` son produit, `last(t)` sa
dernière station et `L_t` sa distance interne.

Premier voyage du véhicule `k` :

```text
cost(start[k], t) = distance(garage[k], d_t)
                  + changeover(initialProduct[k], p_t)
                  + L_t
```

Entre deux voyages :

```text
cost(t, u) = distance(last(t), d_u)
           + changeover(p_t, p_u)
           + L_u
```

Retour final :

```text
cost(t, end[k]) = distance(last(t), garage[k])
```

Le coût diagonal est ajouté à chaque rechargement du même produit. La fonction
objectif est :

```text
totalCost = somme(routeCosts[k])
          = distance totale + coûts de préparation/chargement
```

## 9. Propagation de `TransitionCost`

La contrainte
[`TransitionCost`](../src/main/java/org/maxicp/cp/engine/constraints/seqvar/TransitionCost.java)
ne se limite plus à calculer le coût lorsque la route est fixée.

Pour les matrices non négatives, elle combine trois relaxations :

- la somme des arcs sortants minimaux des nœuds requis ;
- la somme des arcs entrants minimaux des nœuds requis ;
- les plus courts chemins entre membres consécutifs déjà insérés.

La borne inférieure est le maximum de ces trois valeurs. Les plus courts chemins
préservent la validité de la borne lorsque la matrice dirigée des coûts de
changement ne respecte pas l'inégalité triangulaire.

Pour chaque position d'insertion, la contrainte calcule également une borne
conditionnelle. Une position est supprimée avec `notBetween` lorsque cette borne
dépasse la borne supérieure courante de l'objectif.

Les nœuds seulement possibles ne contribuent pas à la borne globale : ils peuvent
encore être exclus. Les matrices contenant des coûts négatifs conservent le calcul
exact à la fermeture de la route, mais n'utilisent pas ces bornes partielles.

## 10. Recherche

La recherche sélectionne d'abord le voyage ayant le moins de positions
d'insertion dans l'ensemble des routes. Parmi ses positions, elle préfère le
plus petit détour :

```text
cost(predecessor, trip) + cost(trip, successor)
                       - cost(predecessor, successor)
```

Le premier branchement effectue l'insertion ; le second interdit cette position.
Chaque solution améliorante est immédiatement écrite, ce qui permet de conserver
un incumbent lorsque le processus est interrompu par une limite de temps externe.

## 11. Garanties et limites

Pour toute solution produite, le modèle garantit :

- la satisfaction exacte des demandes construites ;
- le respect des stocks consommés pendant la construction ;
- la capacité du véhicule pour chaque voyage ;
- un seul produit et un seul chargement par voyage ;
- au plus une visite station-produit par véhicule ;
- une affectation unique de chaque voyage ;
- une route connectée entre le garage de départ et le même garage de retour ;
- les coûts initiaux, successifs et diagonaux de chargement ;
- une sortie conforme à [`solution_format.md`](solution_format.md).

Lorsque `buildTrips` termine avec succès, le témoin de construction garantit en
plus l'existence d'au moins une affectation des voyages respectant les capacités
et les visites distinctes. Une incohérence immédiate lors de la publication de
ces contraintes signalerait donc une régression de la construction, et non une
infaisabilité démontrée de l'instance originale.

Restent fixés heuristiquement :

- le nombre et la quantité des fractions ;
- les dépôts fournisseurs ;
- le regroupement des livraisons ;
- l'ordre et l'extrémité finale des stations de chaque voyage.

Le modèle CP est exact uniquement dans cet espace restreint. Le modèle
`MPVRP.java` doit être utilisé lorsque ces décisions doivent rester dans l'espace
de recherche exact.

## 12. Exécution

Exécution directe :

```text
MPVRPSeqVar <instance.dat> [solution.dat] [nearest-neighbor|2-opt]
```

`2-opt` est utilisé par défaut.

Le script de campagne permet de choisir le solveur :

```bash
./solve_mpvrp_cc.sh hybrid-opt [secondes]
./solve_mpvrp_cc.sh hybrid-nn  [secondes]
./solve_mpvrp_cc.sh exact      [secondes]
```

Les anciens raccourcis `opt` et `nn` restent acceptés. Le mode exact transmet la
limite à `MPVRP`, qui peut alors sortir proprement et préciser si l'optimalité est
certifiée. Un garde-fou externe reste actif pour arrêter une JVM qui ne répondrait
pas.

Les tests principaux sont :

```text
src/test/java/org/maxicp/cp/examples/raw/mpvrp/MPVRPSeqVarTest.java
src/test/java/org/maxicp/cp/engine/constraints/seqvar/TransitionCostTest.java
src/test/java/org/maxicp/cp/examples/raw/mpvrp/MPVRPTest.java
```

La suite `MPVRPSeqVarTest` vérifie notamment :

- le fractionnement entre plusieurs dépôts ;
- le repli `PRESERVE_LARGE_STOCKS` ;
- le regroupement avec capacités hétérogènes ;
- l'existence d'une affectation distincte pour les instances 003, 007 et 009 ;
- le score 2-opt tenant compte de la continuation ;
- la conservation des demandes, des stocks et des capacités sur une instance du
  benchmark ;
- la cohérence entre le coût matriciel et la solution développée.
