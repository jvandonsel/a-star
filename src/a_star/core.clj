;;; Simple A* path finder
;;; Jim Van Donsel, November 2016
(ns a-star.core)


(def MAP  [[1 1 1 1 1 1 1 1 1 1]
           [1 1 1 1 1 1 1 1 1 1]
           [0 0 0 0 0 0 0 1 1 1]
           [1 1 1 1 1 1 1 1 1 1]
           [1 1 1 1 1 0 0 0 0 0]
           [1 1 1 1 1 0 1 1 1 1]
           [1 1 1 1 1 1 1 1 1 1]
           [1 1 1 1 1 0 0 0 0 1]
           [1 0 0 0 0 0 1 1 0 0]
           [1 1 1 1 1 1 1 1 1 1]])

(def START [0 0])
(def GOAL  [9 9])
(def HEIGHT (count MAP))
(def WIDTH (count (first MAP)))

;; Our Path structure
(defrecord Path [locs                 ;; list of nodes in the path
                 incremental-cost     ;; cost so far
                 estimated-cost       ;; estimated cost to goal
                 current-point        ;; leading point in our path
                 ] )

;; Returns the value at the given map location
(defn map-at
  ([x y]
   (get (get MAP y) x)
   )
  ([loc]
   (get (get MAP (second loc)) (first loc))
   )
  )

;; Returns the Manhattan distance between 2 points
(defn manhattan [p1 p2]
  (apply + (map #(Math/abs %) (map - p1 p2)))
  )


;; Dumps the map with the path highlighted with stars
(defn dump-path [path]
  ;; Replace map entries with stars with they coincide with the path
  (let [annotated        (for [y (range HEIGHT) x (range WIDTH)]
                           (if (some #(= % [x y]) (:locs  path)) \* (map-at x y))
                           )]
    ;; print each entry in the map
    (doseq [y (range HEIGHT)]
      (doseq [x (range WIDTH)]
        (print (nth annotated (+ x (* y WIDTH))) " ")
        )
      (println)
      )))


;; Returns all legal neighbors for the given location (i.e. nodes where there is a '1' in the map.
(defn get-neighbors [loc] 
  (if (nil? loc)
    []
    (let [[x y] loc
          n [x (dec y)]
          s [x (inc y)]
          e [(inc x) y]
          w [(dec x) y]
          ]
      (filter #(= 1 (map-at %)) [n s e w])
      ))
  )


;; Our heuristic cost function from the given location to the GOAL.
(defn heuristic [loc]  (manhattan loc GOAL)  )


;; Given a path, tries all the neighbors and returns their new paths with adjusted costs.
;; Excludes nodes that have been visited already.
(defn find-next-paths [path visited-nodes-set]
  (let [
        current-point              (:current-point path)
        previous-points            (:locs path)
        neighbors                  (get-neighbors current-point)
        current-cost               (:incremental-cost path)
        incremental-cost           (inc current-cost)
        filtered-neighbors         (filter #(not (get visited-nodes-set %)) neighbors)
        ]
    (map
     (fn[loc]
       (Path. (conj previous-points loc) incremental-cost (+ incremental-cost (heuristic loc)) loc))
       filtered-neighbors)
     )
  )

;; Applies the A* algorithm to the given sorted queue of paths.
;; Assume the lowest cost path is always at the front of the queue.
(defn find-path [queue visited-nodes-set]
  (let [path-under-test (first queue) ]
    (cond
      ;; no path?
      (empty? queue) nil

      ;; success?
      (= GOAL (:current-point path-under-test))      (first queue)

      ;; Try the first entry in our queue...
      :default          (let [
                              new-paths         (find-next-paths path-under-test visited-nodes-set)
                              new-locs          (map :current-point new-paths)
                              updated-queue     (sort-by :estimated-cost (concat new-paths (rest queue)))
                              updated-visited   (into visited-nodes-set new-locs)
                              size              (count queue)
                              ]
                          
                          (println "find-path: size=" size " newpaths=" (count new-paths) " queue=" queue )
                          (dump-path path-under-test)
                          ;; Recurse!
                          (recur updated-queue updated-visited)
                          ))))


;; Runs the A* algorithm for our MAP, from START to GOAL, and plots the result
(defn -main []
  (let [
        
        initial-path   (Path.  [START] 0 (heuristic START) START)
        initial-queue  [initial-path]
        visited-nodes-set  #{START}
        best-path      (find-path initial-queue visited-nodes-set)]

    (if (nil? best-path)
      (println "\nNo path found")
      (doall (println "\nFINAL PATH:") (dump-path best-path)))
    )
  )
