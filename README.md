# Oblig-2 Dokumentasjon
Under ligger Hele funksjonen for A-stjerne algoritmen. 
```py
    def AStarSearch(self, start_vertex_name = None, target_vertex_name = None) -> list:
        # Check to see that startvertex is in Graph
        if start_vertex_name not in self.vertecies:
            raise KeyError("Start node not present in graph")
        
        from heapdict import heapdict
        self.initPygame()
        count = 0 # Amount of Nodes visited, this is just a performance checker, i've done some small fine tuning.
        D = 1.05 # Fine tuning part... look in documentation for reasoning and source. If this A-Star algorithm is to be used on another map, reset to 1.00

        # Setting Vertex values for g and f
        for key in self.vertecies:
            self.vertecies[key].g = float('inf')

        # Initial value for the first vertex. g = 0 because we have not moved anywhere.
        vertex:Vertex = self.vertecies[start_vertex_name]
        vertex.g = 0

        priqueue = heapdict() # Queue initialized
        priqueue[vertex] = vertex.f # Start Vertex added to queue

        while priqueue:
            count += 1 # Another Vertex/Node has been opened.

            # Get the element with lowest priority (i.e. lowest f value)
            vertex = priqueue.popitem()[0]

            if vertex.name == target_vertex_name: # If we've found the goal, exit while loop and finish up.
                break

            # color the current value and the start vertex and the end vertex it's appropriate colors.
            self.pygameState(vertex, self.GREEN)
            self.pygameState(self.vertecies[start_vertex_name], self.BLUE)
            self.pygameState(self.vertecies[target_vertex_name], self.RED)

            # Look through and add the neighours that are worth visiting to the heapdict.
            for neighbour in vertex.adjecent:
                if neighbour.vertex.g > vertex.g + neighbour.weight:
                    neighbour.vertex.h = self.heuristics(neighbour.vertex.name, target_vertex_name) * D
                    neighbour.vertex.g = vertex.g + neighbour.weight
                    neighbour.vertex.f = neighbour.vertex.g + neighbour.vertex.h
                    neighbour.vertex.previous = vertex

                    priqueue[neighbour.vertex] = neighbour.vertex.f
                    self.pygameState(neighbour.vertex, self.PINK)

            self.pygameState(vertex, self.LIGHTGREY) # Mark this Vertex as visited in pygame.
            
        # Target has been found, show statistics and return path.
        print('Amount of nodes visited:', count)

        length = 0
        for n in self.getPath(start_vertex_name, target_vertex_name): # Update pygame with path found.
            self.pygameState(n, self.DARKGREEN)
            length += 1

        print(f"Path length: {length}")
        return self.getPath(start_vertex_name, target_vertex_name) # Not used?
```

Jeg kommer til å gå gjennom koden noen linjer av gangen.

Blokken under inneholder en simpel test som Dijkstra algoritmen i samme fil har. Jeg tenkte det kunne være fint å ha dette som felles 'sanity check' mellom algoritmene. Neste er de imports og oppstart av pygame, count er en variabel jeg bruker for å sjekke antall nodes jeg har søkt gjennom fra start. Dette er mer for egen nysgjerighet enn noe annet. Den siste variabelen 'D' kommer jeg til å snakke mer om senere i dokumentasjonen, det har å gjøre med 'fin justering' av A-Star til et gitt kart.

```py
        # Check to see that startvertex is in Graph
        if start_vertex_name not in self.vertecies:
            raise KeyError("Start node not present in graph")
        
        from heapdict import heapdict
        self.initPygame()
        count = 0 # Amount of Nodes visited, this is just a performance checker, i've done some small fine tuning.
        D = 1.05 # Fine tuning part... look in documentation for reasoning and source. If this A-Star algorithm is to be used on another map, reset to 1.00
```

For-loopen er for å sette opp alle nodene slik at vi vet at de ikke har blitt besøkt. Dette er for A-stjerne og faktiske vekten for å gå fra denne til en annen node litter i Edge klassen.
Så henter vi ut start noden fra self.vertecies og setter g til 0. Siden det er start punktet har vi ikke gått noe sted.
Opprett kø og legg start noden på kø.

```py
        # Setting Vertex values for g and f
        for key in self.vertecies:
            self.vertecies[key].g = float('inf')

        # Initial value for the first vertex. g = 0 because we have not moved anywhere.
        vertex:Vertex = self.vertecies[start_vertex_name]
        vertex.g = 0

        priqueue = heapdict() # Queue initialized
        priqueue[vertex] = vertex.f # Start Vertex added to queue
```

Så kommer hoved loopen, den står for hele jobben til A-stjerne.

```py
        while priqueue:
            count += 1 # Another Vertex/Node has been opened.

            # Get the element with lowest priority (i.e. lowest f value)
            vertex = priqueue.popitem()[0]

            if vertex.name == target_vertex_name: # If we've found the goal, exit while loop and finish up.
                break

            # color the current value and the start vertex and the end vertex it's appropriate colors.
            self.pygameState(vertex, self.GREEN)
            self.pygameState(self.vertecies[start_vertex_name], self.BLUE)
            self.pygameState(self.vertecies[target_vertex_name], self.RED)

            # Look through and add the neighours that are worth visiting to the heapdict.
            for neighbour in vertex.adjecent:
                if neighbour.vertex.g > vertex.g + neighbour.weight:
                    neighbour.vertex.h = self.heuristics(neighbour.vertex.name, target_vertex_name) * D
                    neighbour.vertex.g = vertex.g + neighbour.weight
                    neighbour.vertex.f = neighbour.vertex.g + neighbour.vertex.h
                    neighbour.vertex.previous = vertex

                    priqueue[neighbour.vertex] = neighbour.vertex.f
                    self.pygameState(neighbour.vertex, self.PINK)

            self.pygameState(vertex, self.LIGHTGREY) # Mark this Vertex as visited in pygame.
```

Mens køen ikke er tom så fortsetter vi. Grunnen til at det ikke er en 'while True' loop er for hvis noen har klart å gi inn en node som ikke finnes i grafen, selv om dette blir testet for tidligere i koden er det bedre å være på den sikre siden.

Som sakt tidligere er count en teller for antallet noder som har blitt gått gjennom (inkludert duplikater).

Etter det hentes neste element i køen og sjekker om det er noden vi leter etter. Hvis ja går vi ut av while loopen og rydder opp før return av 'self.GetPath()'

```py
        while priqueue:
            count += 1 # Another Vertex/Node has been opened.

            # Get the element with lowest priority (i.e. lowest f value)
            vertex = priqueue.popitem()[0]

            if vertex.name == target_vertex_name: # If we've found the goal, exit while loop and finish up.
                break
```

Oppdater fargen til noden som er neste fra køen, og behold fargen av start- og slutt-noden.

```py
            # color the current value and the start vertex and the end vertex it's appropriate colors.
            self.pygameState(vertex, self.GREEN)
            self.pygameState(self.vertecies[start_vertex_name], self.BLUE)
            self.pygameState(self.vertecies[target_vertex_name], self.RED)
```

Dette er hoved delen av A-stjerne, her henter vi alle naboene til noden fra køen.
'if setningen' kan tolkes som følgende: hvis nabo_node.g er større enn nåværende node.g + hva det koster å gå til nabonoden så fortsett.
Siden alle nodene starter med g = float('inf') så vil alle noder som ikke har blitt besøkt passere denne 'if setningen'.

Så kalkuleres h, g og f
h er hauristikken gange en tidligere satt verdi. Hauristikk er hvor langt det er fra nabo_noden til slutt noden i luft linje.
D som er en tidligere satt verdi brukes for fin justering av algoritmen, dette blir jeg å snakke om til slutt i gjennomgangen av koden.

g er kostnaden fra noden vi er i nå til start noden, med andre ord, hva det har kostet å komme seg hit.
f er g + h, dette er priorotets verdien til noden som legges i køen.
Når en node har blitt sett på så settes fargen til rosa og vi kan da se alle nodene på skjermen som ligger i køen til en hver tid.

til slutt når vi er ferdig med å se på en node så fargelegges den grå for å vise at vi har besøkt den har gjort regning på nodene som skal legges i køen.

```py
            # Look through and add the neighours that are worth visiting to the heapdict.
            for neighbour in vertex.adjecent:
                if neighbour.vertex.g > vertex.g + neighbour.weight:
                    neighbour.vertex.h = self.heuristics(neighbour.vertex.name, target_vertex_name) * D
                    neighbour.vertex.g = vertex.g + neighbour.weight
                    neighbour.vertex.f = neighbour.vertex.g + neighbour.vertex.h
                    neighbour.vertex.previous = vertex

                    priqueue[neighbour.vertex] = neighbour.vertex.f
                    self.pygameState(neighbour.vertex, self.PINK)

            self.pygameState(vertex, self.LIGHTGREY) # Mark this Vertex as visited in pygame.
```

Til slutt er det litt statisikk som blir printet ut, dette har jeg brukt for å gjøre fin justering av A-stjerne algoritmen til et bestemt kart.
For-loopen tegner opp veien fra start til mål som algoritmen har funnet frem til.

length er også en del av 'statistikk' for å vise til hvor mange steg det er fra start til mål.

Til slutt returneres listen av path hvis man vil bearbeide resultatet av algoritmen.

```py
        # Target has been found, show statistics and return path.
        print('Amount of nodes visited:', count)

        length = 0
        for n in self.getPath(start_vertex_name, target_vertex_name): # Update pygame with path found.
            self.pygameState(n, self.DARKGREEN)
            length += 1

        print(f"Path length: {length}")
        return self.getPath(start_vertex_name, target_vertex_name) # Not used?
```

## Hvordan kom jeg frem til det jeg har gjort.
Jeg startet med å kopiere koden vi fikk fra Dijkstra og bearbeidet denne som best mulig.
Endringene kommer i form av: oppsett av node.g slik at vi vet at noden ikke har blitt besøkt, prioritets verdien for et element i køen, enkel sjekk når vi har kommet frem til målet og kalkulering av naboer sine verdier og legger dem på kø.

I tilegg har jeg lagt ved count og length for å vise til hvor lang tid algoritmen tar, siden tiden det tar vil variere fra PC til PC og gang til gang koden kjøres valgte jeg å heller se på antall noder besøkt og lengden på veien fra start- til slutt-noden.

## Fin justering

Hva skjer hvis jeg endrer på variabelen D?
Når D er større enn 1 er algoritmen grådigere. For å si det på en annen måte blir algoritmen noe som nærmer seg 'Greedy best first'. Ved store verdi endringer, la oss si D = 5 da er det en 'Greedy best first' algoritme.
Hvis vi går nærmere 0 blir algoritmen nærmere Dijkstra. Ved 0 er den det samme som Dijkstra.

Det gir mening hvorfor algoritmen besøker færre noder hvis det er litt grådigere.
Jeg kom frem til at de følgende verdiene fungerer bra på de 2 100x100 matrisene:

D = 1.03 for det nest siste kartet.
D = 1.05 for det siste kartet.

Men, dersom dette algoritmen skal kjøres på et stort kart og et vektet kart, da vil D = 0 gi best resultat tror jeg. Fordi da er vi tilbake til A-stjerne standard algoritmen.

## Forskjellige søke-algoritmer og forksjeller mellom noen av dem.

A star er den mest effektive uansett hvordan kartet ser ut. Det A-stjerne garanterer korteste mulig vei selv om den er ganske effektiv.
Greedy best first er et effektivt søk, men kan også lede til lange veier til mål. Med dette mener jeg at veien til mål kan være 10 noder lengre enn korteste vei på det store kartet.
Dijkstra er den minst effektive, fordi den søker gjennom flest noder, men garanterer den korteste vei til mål etter veien er funnet.

Dijkstra er nok et bedre søke verktøy enn 'Greedy best first' fordi det kan ta hånd om vektede grafer. 'Greedy best first' er bedre på simple kart uten vekt og mye ting i veien.
Og A-stjerne er balansen mellom de 2.
