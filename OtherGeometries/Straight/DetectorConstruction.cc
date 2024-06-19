//
// ********************************************************************
// * License and Disclaimer                                           *
// *                                                                  *
// * The  Geant4 software  is  copyright of the Copyright Holders  of *
// * the Geant4 Collaboration.  It is provided  under  the terms  and *
// * conditions of the Geant4 Software License,  included in the file *
// * LICENSE and available at  http://cern.ch/geant4/license .  These *
// * include a list of copyright holders.                             *
// *                                                                  *
// * Neither the authors of this software system, nor their employing *
// * institutes,nor the agencies providing financial support for this *
// * work  make  any representation or  warranty, express or implied, *
// * regarding  this  software system or assume any liability for its *
// * use.  Please see the license in the file  LICENSE  and URL above *
// * for the full disclaimer and the limitation of liability.         *
// *                                                                  *
// * This  code  implementation is the result of  the  scientific and *
// * technical work of the GEANT4 collaboration.                      *
// * By using,  copying,  modifying or  distributing the software (or *
// * any work based  on the software)  you  agree  to acknowledge its *
// * use  in  resulting  scientific  publications,  and indicate your *
// * acceptance of all terms of the Geant4 Software license.          *
// ********************************************************************
//
/// \file optical/OpNovice2/src/DetectorConstruction.cc
/// \brief Implementation of the DetectorConstruction class
//
//
//
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

#include "DetectorConstruction.hh"

#include "DetectorMessenger.hh"
#include "G4RunManager.hh"

#include "G4NistManager.hh"
#include "G4Material.hh"
#include "G4Element.hh"
#include "G4LogicalBorderSurface.hh"
#include "G4LogicalSkinSurface.hh"
#include "G4OpticalSurface.hh"
#include "G4Box.hh"
#include "G4Tubs.hh"
#include "G4SubtractionSolid.hh"
#include "G4IntersectionSolid.hh"
#include "G4VSensitiveDetector.hh"
#include "G4SDManager.hh"
#include "G4UnionSolid.hh"
#include "G4LogicalVolume.hh"
#include "G4ThreeVector.hh"
#include "G4PVPlacement.hh"
#include "G4SystemOfUnits.hh"
#include "G4TwistedBox.hh"

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

DetectorConstruction::DetectorConstruction()
  : G4VUserDetectorConstruction()
  , fDetectorMessenger(nullptr)
{
  fExpHall_x = fExpHall_y = fExpHall_z = 950.0 * cm;
  fTank_x = fTank_y = fTank_z = 1.0 * cm;

  fTank = nullptr;

  fTankMPT    = new G4MaterialPropertiesTable();
  fScintMPT = new G4MaterialPropertiesTable();
  fWorldMPT   = new G4MaterialPropertiesTable();
  fSurfaceMPT = new G4MaterialPropertiesTable();
  fSurfaceMPT2 = new G4MaterialPropertiesTable();

  fSurface2 = new G4OpticalSurface("Surface2");
  fSurface2->SetType(dielectric_dielectric);
  fSurface2->SetFinish(polished);
  fSurface2->SetModel(unified);

  fSurface = new G4OpticalSurface("Surface");
  fSurface->SetType(dielectric_dielectric);
  fSurface->SetFinish(polished);
  fSurface->SetModel(unified);

  const G4int NUM = 6;
  G4double pp[NUM] = {2.0*eV, 2.2*eV, 2.4*eV, 2.6*eV, 2.8*eV, 3.0*eV};
  G4double rindex[NUM] = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
  G4double rindex2[NUM] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  G4double rindex3[NUM] = {1.52, 1.52, 1.52, 1.52, 1.52, 1.52};
  G4double reflectivity[NUM] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

  G4double reflectivity2[NUM] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  G4double tran2[NUM] = {0., 0., 0., 0., 0., 0.};

  G4double tran[NUM] = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
  G4double absorption[NUM] = {3.448*m, 4.082 * m,  6.329 * m,  9.174 * m,  12.346 * m, 13.889 * m};
  fTankMPT->AddProperty("RINDEX", pp, rindex3, NUM);      // was rindex not rindex3
  //fTankMPT->AddProperty("ABSLENGTH", pp, absorption, NUM);
  //fSurfaceMPT->AddProperty("REFLECTIVITY",pp,reflectivity,NUM);
  //fSurfaceMPT->AddProperty("TRANSMITTANCE",pp,tran,NUM);

  fSurfaceMPT2->AddProperty("REFLECTIVITY", pp, reflectivity2, NUM);
  fSurfaceMPT2->AddProperty("TRANSMITTANCE", pp, tran2, NUM);

  fWorldMPT->AddProperty("RINDEX", pp, rindex2, NUM);
  fScintMPT->AddProperty("RINDEX", pp, rindex3, NUM);


  fSurface2->SetMaterialPropertiesTable(fSurfaceMPT2);

  fSurface->SetMaterialPropertiesTable(fSurfaceMPT);

  fTank_LV  = nullptr;
  fWorld_LV = nullptr;
  rect_mid_LV = nullptr;
  cone_LV = nullptr;
  rem_cyl_LV = nullptr;
  rem_cyl2_LV = nullptr;
  rem_cyl3_LV = nullptr;
  rem_cyl4_LV = nullptr;
  rec_box_LV = nullptr;

  // [1.   0.5  0.2  0.1  0.05]
  // [995. 495. 195.  95.  45.   5.]
  // [49.75 24.75  9.75  4.75  2.25  0.25]
  

  fTankMaterial  = G4NistManager::Instance()->FindOrBuildMaterial("G4_GLASS_PLATE");
  fWorldMaterial = G4NistManager::Instance()->FindOrBuildMaterial("G4_AIR");
  // fWorldMaterial = G4NistManager::Instance()->FindOrBuildMaterial("G4_Galactic");
  fScintMaterial = G4NistManager::Instance()->FindOrBuildMaterial("G4_Pyrex_Glass");

  fDetectorMessenger = new DetectorMessenger(this);
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

DetectorConstruction::~DetectorConstruction() {
  delete fTankMPT;
  delete fScintMPT;
  delete fWorldMPT;
  delete fSurfaceMPT;
  delete fSurfaceMPT2;
  delete fSurface;
  delete fSurface2;
  delete fDetectorMessenger;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

G4VPhysicalVolume* DetectorConstruction::Construct()
{
  fTankMaterial->SetMaterialPropertiesTable(fTankMPT);
  fTankMaterial->GetIonisation()->SetBirksConstant(0.126 * mm / MeV);
  fScintMaterial->SetMaterialPropertiesTable(fScintMPT);
  fWorldMaterial->SetMaterialPropertiesTable(fWorldMPT);

  G4double thick = 0.25*cm;
  G4double width = 2.5*cm;
  G4double len = 10*cm;
  G4double l = 100*cm;

  // ------------- Volumes --------------
  // The experimental Hall
  G4Box* world_box = new G4Box("World", fExpHall_x, fExpHall_y, fExpHall_z);

  fWorld_LV = new G4LogicalVolume(world_box, fWorldMaterial, "World", 0, 0, 0);

  world_PV = new G4PVPlacement(0, G4ThreeVector(), fWorld_LV, "World", 0, false, 0, true);

  // Rotations ... need to understand this!
  G4RotationMatrix* Rot = new G4RotationMatrix();
  Rot->rotateZ((bend_ang/2)*deg);

  G4RotationMatrix* Rott = new G4RotationMatrix();
  Rott->rotateZ(90*deg);
    Rott->rotateY(180*deg);


  // //box for PMT view picture
  // //G4Box* photo = new G4Box("Photo", 10*cm, 10*cm, 5*cm);
  // //G4LogicalVolume* photo_LV = new G4LogicalVolume(photo, fTankMaterial, "Photo", 0, 0, 0);
  // //G4PVPlacement* photo_PV = new G4PVPlacement(0, G4ThreeVector(0, 0, -19.9*cm), photo_LV, "Photo", fWorld_LV, false, 0);

  G4Box* rect_mid_curve = new G4Box("TwistedStrip", width, thick, l);
  G4Box* rect_mid_straight = new G4Box("rect_mid2", width, thick, len / 2);
  G4UnionSolid* rect_mid = new G4UnionSolid("rect_mid", rect_mid_straight, rect_mid_curve, 0, G4ThreeVector(0, 0, -l-len/2));
  G4cout << "G4UnionSolid G4ThreeVector: " << "0" << " , " << "0" << " , " << -l-len/2 << G4endl;
  G4cout << "Rotation: " << Rot << G4endl;

  rect_mid_LV = new G4LogicalVolume(rect_mid, fTankMaterial, "rect_mid", 0, 0, 0);
  rect_mid_PV = new G4PVPlacement(0, G4ThreeVector(0, 0, -len/2), rect_mid_LV, "rect_mid", fWorld_LV, false, 0, true);


  // PMT
  G4Tubs* cone = new G4Tubs("Cone", 0., 2.75*cm, len/2, 0.*deg, 360.0*deg);
  cone_LV = new G4LogicalVolume(cone, fWorldMaterial, "Cone", 0, 0, 0);
  cone_PV = new G4PVPlacement(0, G4ThreeVector(0, 0, -2*l-1.5*len), cone_LV, "Cone", fWorld_LV, false, 0, true);


  // scintillator
  G4Box* scint = new G4Box("Scint", width, thick, len/2);
  G4LogicalVolume* scint_LV = new G4LogicalVolume(scint, fTankMaterial, "Scint", 0, 0, 0);
  scint_PV = new G4PVPlacement(0, G4ThreeVector(0, 0, len/2), scint_LV, "Scint", fWorld_LV, false, 0, true);



  // ------------- Surface --------------

  // G4LogicalBorderSurface* surface1 =
  //  new G4LogicalBorderSurface("Surface1", rect_mid_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface2 =
  //  new G4LogicalBorderSurface("Surface2", rect_left_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface3 =
  //  new G4LogicalBorderSurface("Surface3", rect_right_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface4 =
  //  new G4LogicalBorderSurface("Surface4", union2_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface5 =
  //  new G4LogicalBorderSurface("Surface5", union2_PV2, world_PV, fSurface);

  //G4LogicalBorderSurface* surface6 =
  //  new G4LogicalBorderSurface("Surface6", intersect_rect_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface7 =
  //  new G4LogicalBorderSurface("Surface7", intersect_rect2_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface8 =
  //  new G4LogicalBorderSurface("Surface8", rect_left2_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface9 =
  //  new G4LogicalBorderSurface("Surface9", rect_right2_PV, world_PV, fSurface);

  // G4LogicalBorderSurface* surface10 =
  //  new G4LogicalBorderSurface("Surface10", cone_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface11 =
  //  new G4LogicalBorderSurface("Surface11", union3_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface12 =
  //  new G4LogicalBorderSurface("Surface12", union3_PV2, world_PV, fSurface);

  //G4LogicalBorderSurface* surface13 =
  //  new G4LogicalBorderSurface("Surface13", union4_PV, world_PV, fSurface);

  //G4LogicalBorderSurface* surface14 =
  //  new G4LogicalBorderSurface("Surface14", union4_PV2, world_PV, fSurface);

  // G4LogicalBorderSurface* surface15 =
  //  new G4LogicalBorderSurface("Surface15", scint_PV, world_PV, fSurface);

  //G4OpticalSurface* opticalSurface = dynamic_cast<G4OpticalSurface*>(
  //  surface->GetSurface(fTank, world_PV)->GetSurfaceProperty());
  //G4cout << "******  opticalSurface->DumpInfo:" << G4endl;
  //if(opticalSurface)
  //{
  //  opticalSurface->DumpInfo();
  //}
  //G4cout << "******  end of opticalSurface->DumpInfo" << G4endl;

  return world_PV;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::SetSurfaceSigmaAlpha(G4double v)
{
  fSurface->SetSigmaAlpha(v);
  G4RunManager::GetRunManager()->GeometryHasBeenModified();

  G4cout << "Surface sigma alpha set to: " << fSurface->GetSigmaAlpha()
         << G4endl;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::SetSurfacePolish(G4double v)
{
  fSurface->SetPolish(v);
  G4RunManager::GetRunManager()->GeometryHasBeenModified();

  G4cout << "Surface polish set to: " << fSurface->GetPolish() << G4endl;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::AddTankMPV(const G4String& prop,
                                      G4MaterialPropertyVector* mpv)
{
  fTankMPT->AddProperty(prop, mpv);
  G4cout << "The MPT for the box is now: " << G4endl;
  fTankMPT->DumpTable();
  G4cout << "............." << G4endl;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::AddWorldMPV(const G4String& prop,
                                       G4MaterialPropertyVector* mpv)
{
  fWorldMPT->AddProperty(prop, mpv);
  G4cout << "The MPT for the world is now: " << G4endl;
  fWorldMPT->DumpTable();
  G4cout << "............." << G4endl;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::AddSurfaceMPV(const G4String& prop,
                                         G4MaterialPropertyVector* mpv)
{
  fSurfaceMPT->AddProperty(prop, mpv);
  G4cout << "The MPT for the surface is now: " << G4endl;
  fSurfaceMPT->DumpTable();
  G4cout << "............." << G4endl;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::AddTankMPC(const G4String& prop, G4double v)
{
  fTankMPT->AddConstProperty(prop, v);
  G4cout << "The MPT for the box is now: " << G4endl;
  fTankMPT->DumpTable();
  G4cout << "............." << G4endl;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::AddWorldMPC(const G4String& prop, G4double v)
{
  fWorldMPT->AddConstProperty(prop, v);
  G4cout << "The MPT for the world is now: " << G4endl;
  fWorldMPT->DumpTable();
  G4cout << "............." << G4endl;
}
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::AddSurfaceMPC(const G4String& prop, G4double v)
{
  fSurfaceMPT->AddConstProperty(prop, v);
  G4cout << "The MPT for the surface is now: " << G4endl;
  fSurfaceMPT->DumpTable();
  G4cout << "............." << G4endl;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::SetWorldMaterial(const G4String& mat)
{
  G4Material* pmat = G4NistManager::Instance()->FindOrBuildMaterial(mat);
  if(pmat && fWorldMaterial != pmat)
  {
    fWorldMaterial = pmat;
    if(fWorld_LV)
    {
      fWorld_LV->SetMaterial(fWorldMaterial);
      fWorldMaterial->SetMaterialPropertiesTable(fWorldMPT);
    }
    G4RunManager::GetRunManager()->PhysicsHasBeenModified();
    G4cout << "World material set to " << fWorldMaterial->GetName() << G4endl;
  }
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DetectorConstruction::SetTankMaterial(const G4String& mat)
{
  G4Material* pmat = G4NistManager::Instance()->FindOrBuildMaterial(mat);
  if(pmat && fTankMaterial != pmat)
  {
    fTankMaterial = pmat;
    if(fTank_LV)
    {
      fTank_LV->SetMaterial(fTankMaterial);
      fTankMaterial->SetMaterialPropertiesTable(fTankMPT);
      fTankMaterial->GetIonisation()->SetBirksConstant(0.126 * mm / MeV);
    }
    G4RunManager::GetRunManager()->PhysicsHasBeenModified();
    G4cout << "Tank material set to " << fTankMaterial->GetName() << G4endl;
  }
}

void DetectorConstruction::SetBendRadius(G4double radius)
{
  bend_rad = radius*cm;
  G4RunManager::GetRunManager()->PhysicsHasBeenModified();
  world_PV = DetectorConstruction::Construct();
}

void DetectorConstruction::SetBendAngle(G4double angle)
{
  bend_ang = angle*deg;
  G4RunManager::GetRunManager()->PhysicsHasBeenModified();
  world_PV = DetectorConstruction::Construct();
}
