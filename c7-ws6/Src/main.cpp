#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkPNGReader.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkWarpScalar.h>
#include <vtkLookupTable.h>
#include <vtkImageData.h>
#include <vtkInteractorStyle.h>
#include <vtkDelaunay3D.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyDataReader.h>
#include <vtkCallbackCommand.h>

#include <vtkCleanPolyData.h>

#include "iostream"
#include "string"

#define vtkSPtr vtkSmartPointer
#define vtkSPtrNew(Var, Type) vtkSPtr<Type> Var = vtkSPtr<Type>::New();


using namespace std;

/**
 * Reads a PNG grayscale image and generates and Delaunay triangular mesh vtkFile
 * The vtk PolyData file is persisted for later use
 * @param imgName path and name of the pngImage e.g ../Img/myMap.png , to generate mesh from
 * @param meshName path and name of the pngImage e.g ../Img/myMap.vtk, to persist mesh into
 * @param step real height of the height map for normalization
 */
void generateDelaunayMesh(string imgName = "", string meshName = "", double maxHeight = 0) {

	if ( imgName.empty() || meshName.empty ()) {
		cout << "generateDelaunayMesh - image name or mesh name empty"<<endl;
		return;
	} else {
		cout << "generateDelaunayMesh - image name or mesh name NOT empty"<<endl;
		/* Reads the image with the grey scale height values */
		vtkSPtrNew(reader, vtkPNGReader);
		reader->SetFileName (imgName.c_str ());
		reader->Update();
		double lo = reader->GetOutput ()->GetScalarRange ()[ 0 ];
		double hi = reader->GetOutput ()->GetScalarRange ()[ 1 ];
		cout << "generateDelaunayMesh - vtkPNGReader" << endl;

		//vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
		//cleaner->SetInputConnection(reader->GetOutputPort());

		/* Extracts geometry from a structured points' dataset (Image) */
		vtkSPtrNew(surface, vtkImageDataGeometryFilter);
		surface->SetInputConnection (reader->GetOutputPort ());
		surface->Update();
		cout << "generateDelaunayMesh - vtkImageDataGeometryFilter" << endl;


        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//!!!  Generate delaunay Mesh from geometry above  
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		


		vtkSPtrNew(delaunay, vtkDelaunay3D);
		delaunay->SetInputConnection(surface->GetOutputPort());
		cout << "generateDelaunayMesh - vtkDelaunay3D - before update" << endl;
		delaunay->Update();
		cout << "generateDelaunayMesh - vtkDelaunay3D" << endl;

		vtkSPtrNew(warpGemoetry, vtkWarpScalar);
		warpGemoetry->SetInputConnection(delaunay->GetOutputPort());
		double step = maxHeight / hi;  // divide mountain height by image gray value (normalize)
		warpGemoetry->SetScaleFactor(step);
		warpGemoetry->UseNormalOn();
		warpGemoetry->SetNormal(0, 0, 1);
		warpGemoetry->Update();
		cout << "generateDelaunayMesh - vtkWarpScalar" << endl;

		vtkSPtrNew(writer, vtkPolyDataWriter);
		writer->SetInputConnection (warpGemoetry->GetOutputPort());
		writer->SetFileName (meshName.c_str());
		writer->Write();
		cout << "generateDelaunayMesh - image vtk written"<<endl;

	}
}

/*
* Custom function for key event
*/
void keyboardCallbackFunction (vtkObject *caller, unsigned long eventId, void *clientData, void *callData) {
	vtkSPtrNew(interactor, vtkRenderWindowInteractor);
	interactor = static_cast<vtkRenderWindowInteractor *>(caller);
	string keyPressed = interactor->GetKeySym ();
	if ( keyPressed == "p" ||  keyPressed == "P") {
	    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//!!! Locate and print the Vertex ID pointed by the cursor
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//! vtkPointPicker Class Reference - VTK
		//! 
	}
}

int main() {

	generateDelaunayMesh ("Mountain.png","Mountain.vtk",100.0);

	/** Map  ***********************************/

	vtkSPtrNew(reader, vtkPolyDataReader);
	reader->SetFileName ("Mountain.vtk");
	reader->Update ();
	double lo = reader->GetOutput ()->GetScalarRange ()[ 0 ];
	double hi = reader->GetOutput ()->GetScalarRange ()[ 1 ];

	// Map scalar (height) values into color (RGBA)
	vtkSPtrNew(lut, vtkLookupTable);
	lut->SetHueRange (0.65, 0);
	lut->SetSaturationRange (0.9, 0);
	lut->SetValueRange (0.25, 1.0);

	// Maps polygonal data to graphics primitives for later GPU rendering
	vtkSPtrNew(mapMapper, vtkPolyDataMapper);
	mapMapper->SetInputConnection (reader->GetOutputPort ());
	mapMapper->SetScalarRange (lo, hi);
	mapMapper->SetLookupTable (lut);

	vtkSPtrNew(mapActor, vtkActor);
	mapActor->SetMapper (mapMapper);

	vtkSPtrNew(colors, vtkNamedColors);
	mapActor->GetProperty ()->SetSpecular (.3);
	mapActor->GetProperty ()->SetSpecularPower (3);
	mapActor->GetProperty ()->SetRepresentationToWireframe ();

	/** Path  ***********************************/

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!! Compute the shortest path among 2 vertices and display them 
	//!!! on the screen 
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 

	/** Common visualization ***********************************/

	vtkSPtrNew(renderer, vtkRenderer);
	renderer->AddActor (mapActor);
	renderer->SetBackground (0.5, 0.6, 0.7);

	vtkSPtrNew(keyboardCallback, vtkCallbackCommand);
	keyboardCallback->SetCallback (keyboardCallbackFunction);

	vtkSPtrNew(renderWindow, vtkRenderWindow);
	renderWindow->AddRenderer (renderer);

	vtkSPtrNew(interactor, vtkRenderWindowInteractor);
	interactor->SetRenderWindow (renderWindow);
	interactor->AddObserver (vtkCommand::KeyPressEvent, keyboardCallback);

	renderer->ResetCamera ();
	renderWindow->Render ();
	interactor->Start ();

	return 0;
}





