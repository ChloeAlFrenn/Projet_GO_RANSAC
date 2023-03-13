package main

import (
	"bufio"
	"fmt"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"
)

type Point3D struct {
	X float64
	Y float64
	Z float64
}

type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
}

type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

// reads an XYZ file and returns a slice of Point3D
func ReadXYZ(filename string) []Point3D {
	file, err := os.Open(filename)
	//gestion d'erreurs
	if err != nil {
		fmt.Println("Erreur de lecture:", err)
		return nil
	}
	defer file.Close()

	//create the slice
	var points []Point3D
	scanner := bufio.NewScanner(file)
	scanner.Scan() // skip the first line x,y,z

	//while en go
	for scanner.Scan() {
		ligne := strings.Fields(scanner.Text())
		if len(ligne) == 0 {
			// ligne vide plus d'elements
			break
		}
		if len(ligne) != 3 {
			// pas xyz dans la ligne
			continue
		}
		x, _ := strconv.ParseFloat(ligne[0], 64)
		y, _ := strconv.ParseFloat(ligne[1], 64)
		z, _ := strconv.ParseFloat(ligne[2], 64)

		points = append(points, Point3D{x, y, z})

	}

	return points

}

// saves a slice of Point3D into an XYZ file
func SaveXYZ(filename string, points []Point3D) {
	file, err := os.Create(filename)
	if err != nil {
		fmt.Println("Erreur d'ecriture: ", err)
		return
	}
	defer file.Close()
	for _, point := range points {
		fmt.Fprintf(file, "%f %f %f\n", point.X, point.Y, point.Z)
	}
}

// computes the distance between points p1 and p2
//https://www.engineeringtoolbox.com/distance-relationship-between-two-points-d_1854.html
//d = ((x2 - x1)2 + (y2 - y1)2 + (z2 - z1)2)1/2

func (p1 *Point3D) GetDistance(p2 *Point3D) float64 {
	dx := p2.X - p1.X
	dy := p2.Y - p1.Y
	dz := p2.Z - p1.Z
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// computes the plane defined by a slice of 3 points
// same formula as constructor in class PLANE3D in java part
func GetPlane(points []Point3D) Plane3D {
	p1 := points[0]
	p2 := points[1]
	p3 := points[2]
	//vectors between the points
	p1p2 := Point3D{p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z}
	p1p3 := Point3D{p3.X - p1.X, p3.Y - p1.Y, p3.Z - p1.Z}
	//cross product of the vectors
	normal := Point3D{(p1p2.Y * p1p3.Z) - (p1p2.Z * p1p3.Y), (p1p2.Z * p1p3.X) - (p1p2.X * p1p3.Z), (p1p2.X * p1p3.Y) - (p1p2.Y * p1p3.X)}
	//form a + b + c + d = 0
	a := normal.X
	b := normal.Y
	c := normal.Z
	d := -((a * p1.X) + (b * p1.Y) + (c * p1.Z))
	return Plane3D{a, b, c, d}
}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64, percentageOfPointsOnPlane float64) int {
	num := math.Log(1 - confidence)
	den := math.Log(1 - (math.Pow(percentageOfPointsOnPlane, 3)))
	it := num / den
	iterations := int(it)
	return iterations
}

// computes the support of a plane in a slice of points
// find the plane and get the size
func GetSupport(plane Plane3D, points []Point3D, eps float64) Plane3DwSupport {
	//get a point on the plane
	var increment int
	var x, y, z float64
	if plane.A != 0 {
		x = 0
		y = (-plane.C*z - plane.D) / plane.B
		z = (-plane.B*y - plane.D) / plane.C
	} else if plane.B != 0 {
		y = 0
		x = (-plane.C*z - plane.D) / plane.A
		z = (-plane.A*x - plane.D) / plane.C
	} else {
		z = 0
		x = (-plane.B*y - plane.D) / plane.A
		y = (-plane.A*x - plane.D) / plane.B
	}

	temp := Point3D{x, y, z}

	for _, point := range points {
		if temp.GetDistance(&point) < eps {
			increment++
		}
	}
	return Plane3DwSupport{plane, increment}

}

// extracts the points that supports the given plane
// and returns them in a slice of points
func GetSupportingPoints(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var x, y, z float64
	if plane.A != 0 {
		x = 0.0
		y = (-plane.C*z - plane.D) / plane.B
		z = (-plane.B*y - plane.D) / plane.C
	} else if plane.B != 0 {
		y = 0.0
		z = (-plane.A*x - plane.D) / plane.C
		x = (-plane.C*z - plane.D) / plane.A
	} else {
		z = 0.0
		x = (-plane.B*y - plane.D) / plane.A
		y = (-plane.A*x - plane.D) / plane.B
	}

	temp := Point3D{x, y, z}

	supportingPoints := []Point3D{}
	for _, point := range points {
		if temp.GetDistance(&point) < eps {
			supportingPoints = append(supportingPoints, point)
		}
	}
	return supportingPoints
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var x, y, z float64
	if plane.A != 0 {
		x = 0
		y = (-plane.C*z - plane.D) / plane.B
		z = (-plane.B*y - plane.D) / plane.C
	} else if plane.B != 0 {
		y = 0
		x = (-plane.C*z - plane.D) / plane.A
		z = (-plane.A*x - plane.D) / plane.C
	} else {
		z = 0
		x = (-plane.B*y - plane.D) / plane.A
		y = (-plane.A*x - plane.D) / plane.B
	}

	temp := Point3D{x, y, z}
	newPoints := []Point3D{}
	for _, point := range points {
		if !(temp.GetDistance(&point) < eps) {
			newPoints = append(newPoints, point)
		}
	}
	return newPoints
}

// go run planeRANSAC.go filename confidence percentage eps
//go run planeRANSAC.go PointCloud1.xyz 0.99 0.05 0.5
//go run planeRANSAC.go PointCloud2.xyz 0.99 0.05 0.5
//go run planeRANSAC.go PointCloud3.xyz 0.99 0.05 0.5

func main() {

	start := time.Now()

	numOfGoroutines := 15
	filename := os.Args[1]
	confidence, _ := strconv.ParseFloat(os.Args[2], 64)
	percentage, _ := strconv.ParseFloat(os.Args[3], 64)
	eps, _ := strconv.ParseFloat(os.Args[4], 64)
	var points []Point3D
	points = ReadXYZ(filename)
	bestSupport := Plane3DwSupport{Plane3D{0, 0, 0, 0}, 0}
	numOfIterations := GetNumberOfIterations(confidence, percentage)

	//start of the pipeline
	randCh := RandomPointGenerator(points)
	tripletCh := TripletOfPointsGenerator(points, randCh)
	takeNCh := TakeN(tripletCh, numOfIterations)
	planeCh := PlaneEstimator(takeNCh)
	supportChs := make([]<-chan Plane3DwSupport, numOfGoroutines)
	for i := 0; i < numOfGoroutines; i++ {
		supportChs[i] = SupportingPointFinder(planeCh, points, eps)
	}
	fanInCh := FanIn(supportChs)
	DominantPlaneIdentifier(fanInCh, &bestSupport)
	//end of the pipeline

	pointsOnPlane := GetSupportingPoints(bestSupport.Plane3D, points, eps)
	fname := strings.ReplaceAll(filename, ".xyz", "_p1.xyz")
	SaveXYZ(fname, pointsOnPlane)

	//remove the points
	points = RemovePlane(bestSupport.Plane3D, points, eps)
	//start of the pipeline 2
	bestSupport = Plane3DwSupport{Plane3D{0, 0, 0, 0}, 0}
	randCh2 := RandomPointGenerator(points)
	tripletCh2 := TripletOfPointsGenerator(points, randCh2)
	takeNCh2 := TakeN(tripletCh2, numOfIterations)
	planeCh2 := PlaneEstimator(takeNCh2)
	supportChs2 := make([]<-chan Plane3DwSupport, numOfGoroutines)
	for i := 0; i < numOfGoroutines; i++ {
		supportChs2[i] = SupportingPointFinder(planeCh2, points, eps)
	}
	fanInCh2 := FanIn(supportChs2)
	DominantPlaneIdentifier(fanInCh2, &bestSupport)
	//end of the pipeline 2
	pointsOnPlane2 := GetSupportingPoints(bestSupport.Plane3D, points, eps)
	fname2 := strings.ReplaceAll(filename, ".xyz", "_p2.xyz")
	SaveXYZ(fname2, pointsOnPlane2)

	//remove the points
	points = RemovePlane(bestSupport.Plane3D, points, eps)

	//start of the pipeline 3
	bestSupport = Plane3DwSupport{Plane3D{0, 0, 0, 0}, 0}
	randCh3 := RandomPointGenerator(points)
	tripletCh3 := TripletOfPointsGenerator(points, randCh3)
	takeNCh3 := TakeN(tripletCh3, numOfIterations)
	planeCh3 := PlaneEstimator(takeNCh3)
	supportChs3 := make([]<-chan Plane3DwSupport, numOfGoroutines)
	for i := 0; i < numOfGoroutines; i++ {
		supportChs3[i] = SupportingPointFinder(planeCh3, points, eps)
	}
	fanInCh3 := FanIn(supportChs3)
	DominantPlaneIdentifier(fanInCh3, &bestSupport)
	//end of the pipeline 3
	pointsOnPlane3 := GetSupportingPoints(bestSupport.Plane3D, points, eps)
	fname3 := strings.ReplaceAll(filename, ".xyz", "_p3.xyz")
	SaveXYZ(fname3, pointsOnPlane3)
	points = RemovePlane(bestSupport.Plane3D, points, eps)

	fname0 := strings.ReplaceAll(filename, ".xyz", "_p0.xyz")
	SaveXYZ(fname0, points)

	elapsed := time.Since(start)
	fmt.Printf("With %d goroutines Elapsed time is: %v\n", numOfGoroutines, elapsed)

}

func RandomPointGenerator(points []Point3D) <-chan Point3D {
	randomOut := make(chan Point3D)
	go func() {
		defer close(randomOut)
		rand.Seed(time.Now().UnixNano())
		for {
			randomIndex := rand.Intn(len(points))
			point := points[randomIndex]
			randomOut <- point
		}
	}()
	return randomOut

}

func TripletOfPointsGenerator(points []Point3D, c <-chan Point3D) <-chan [3]Point3D {

	tripletOut := make(chan [3]Point3D)
	go func() {
		defer close(tripletOut)
		for {
			triplet := [3]Point3D{}
			for i := 0; i < 3; i++ {
				triplet[i] = <-c
			}
			tripletOut <- triplet
		}
	}()

	return tripletOut
}

func TakeN(c <-chan [3]Point3D, n int) <-chan [3]Point3D {
	takeOut := make(chan [3]Point3D)
	go func() {
		defer close(takeOut)
		for i := 0; i < n; i++ {
			arr := <-c
			takeOut <- arr
		}
	}()
	return takeOut
}

func PlaneEstimator(c <-chan [3]Point3D) <-chan Plane3D {
	planeOut := make(chan Plane3D)
	go func() {
		defer close(planeOut)
		for triplet := range c {
			plane := GetPlane(triplet[:])
			planeOut <- plane
		}
	}()
	return planeOut
}

func SupportingPointFinder(c <-chan Plane3D, points []Point3D, eps float64) <-chan Plane3DwSupport {
	supportOut := make(chan Plane3DwSupport)
	go func() {
		defer close(supportOut)
		for plane := range c {
			supportingPoints := GetSupport(plane, points, eps)
			supportOut <- supportingPoints
		}
	}()
	return supportOut
}

func FanIn(chans []<-chan Plane3DwSupport) <-chan Plane3DwSupport {
	var wg sync.WaitGroup
	fanOut := make(chan Plane3DwSupport)

	for _, c := range chans {
		wg.Add(1)
		go func(c <-chan Plane3DwSupport) {
			defer wg.Done()
			for plane := range c {
				fanOut <- plane
			}
		}(c)
	}

	go func() {
		wg.Wait()
		close(fanOut)
	}()

	return fanOut
}

func DominantPlaneIdentifier(c <-chan Plane3DwSupport, bestSupport *Plane3DwSupport) {
	for plane := range c {
		if plane.SupportSize > bestSupport.SupportSize {
			*bestSupport = plane
		}
	}
}
