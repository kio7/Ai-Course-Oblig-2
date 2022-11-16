# Oblig-2 Dokumentasjon
Under ligger Hele funksjonen for A-stjerne algoritmen. 
```py
    def AStarSearch(self, startVertexName = None, targetVertexName = None) -> list:
        # Check to see that startvertex is in Graph
        if startVertexName not in self.vertecies:
            raise KeyError("Start node not present in graph")
        
        from heapdict import heapdict
        self.initPygame()
        count = 0 # Amount of Nodes visited, this is just a performance checker, i've done some small fine tuning.
        D = 1.05 # Fine tuning part... look in documentation for reasoning and source. If this A-Star algorithm is to be used on another map, reset to 1.00

        # Setting all Vertex values for h, g and f so it does not need to be done later.
        for key in self.vertecies:
            self.vertecies[key].h = self.heuristics(key, targetVertexName) * D
            self.vertecies[key].g = float('inf')
            self.vertecies[key].f = float('inf')

        # Initial value for the first vertex. g = 0 because we have not moved anywhere.
        vertex:Vertex = self.vertecies[startVertexName]
        vertex.g = 0

        priqueue = heapdict() # Queue initialized
        priqueue[vertex] = vertex.f # Start Vertex added to queue

        while priqueue:
            count += 1 # Another Vertex/Node has been opened.

            # Get the element with lowest priority (i.e. lowest f value)
            vertex = priqueue.popitem()[0]

            if vertex.name == targetVertexName: # If we've found the goal, exit while loop and finish up.
                break

            # color the current value and the start vertex and the end vertex it's appropriate colors.
            self.pygameState(vertex, self.GREEN)
            self.pygameState(self.vertecies[startVertexName], self.BLUE)
            self.pygameState(self.vertecies[targetVertexName], self.RED)

            # Look through and add the neighours that are worth visiting to the heapdict.
            for neighbour in vertex.adjecent:
                if neighbour.vertex.g > vertex.g + neighbour.weight:
                    neighbour.vertex.g = vertex.g + neighbour.weight
                    neighbour.vertex.f = neighbour.vertex.g + neighbour.vertex.h
                    neighbour.vertex.previous = vertex

                    priqueue[neighbour.vertex] = neighbour.vertex.f
                    self.pygameState(neighbour.vertex, self.PINK)

            self.pygameState(vertex, self.LIGHTGREY) # Mark that the Vertex has been visited and finished working on it.
```
Jeg kommer til å gå gjennom koden noen linjer av gangen.

Blokken under inneholder en simpel test som Dijkstra algoritmen i samme fil har. Jeg tenkte det kunne være fint å ha dette som felles 'sanity check' mellom algoritmene. Neste er de imports og oppstart av pygame, count er en variabel jeg bruker for å sjekke antall nodes jeg har søkt gjennom fra start. Dette er mer for egen nysgjerighet enn noe annet. Den siste variabelen 'D' kommer jeg til å snakke mer om senere i dokumentasjonen, det har å gjøre med 'fin justering' av A-Star til et gitt kart.
```py
        # Check to see that startvertex is in Graph
        if startVertexName not in self.vertecies:
            raise KeyError("Start node not present in graph")
        
        from heapdict import heapdict
        self.initPygame()
        count = 0 # Amount of Nodes visited, this is just a performance checker, i've done some small fine tuning.
        D = 1.05 # Fine tuning part... look in documentation for reasoning and source. If this A Start algorythm is to be used on another map, reset to 1.00
```

For loopen er for 

```py
        # Setting all Vertex values for h, g and f so it does not need to be done later.
        for key in self.vertecies:
            self.vertecies[key].h = self.heuristics(key, targetVertexName) * D
            self.vertecies[key].g = float('inf')
            self.vertecies[key].f = float('inf')

        # Initial value for the first vertex. g = 0 because we have not moved anywhere.
        vertex:Vertex = self.vertecies[startVertexName]
        vertex.g = 0

        priqueue = heapdict() # Queue initialized
        priqueue[vertex] = vertex.f # Start Vertex added to queue

```







