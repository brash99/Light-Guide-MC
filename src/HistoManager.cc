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
/// \file optical/OpNovice2/src/HistoManager.cc
/// \brief Implementation of the HistoManager class
//
//
//
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

#include "HistoManager.hh"

#include "G4UnitsTable.hh"

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

HistoManager::HistoManager()
  : fFileName("opnovice2")
{
  Book();
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

HistoManager::~HistoManager() {}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void HistoManager::Book()
{
  // Create or get analysis manager
  G4AnalysisManager* analysisMan = G4AnalysisManager::Instance();
  analysisMan->SetDefaultFileType("root");
  analysisMan->SetFileName(fFileName);
  analysisMan->SetVerboseLevel(1);
  analysisMan->SetActivation(true);  // enable inactivation of histograms

  // Define histograms
  // Default values (to be reset via /analysis/h1/set command)
  G4int n      = 100;
  G4double xmn = 0.;
  G4double xmx = 100.;

  // 0
  analysisMan->CreateH1("0", "dummy", n, xmn, xmx);
  // 1
  analysisMan->CreateH1("Cerenkov spectrum", "Cerenkov spectrum", n, xmn, xmx);
  // 2
  analysisMan->CreateH1("Scintillation spectrum", "Scintillation spectrum", n,
                        xmn, xmx);
  // 3
  analysisMan->CreateH1("Scintillation time",
                        "scintillation photons creation time", n, xmn, xmx);
  // 4
  analysisMan->CreateH1("WLS abs", "WLS absorption spectrum", n, xmn, xmx);
  // 5
  analysisMan->CreateH1("WLS em", "WLS emission spectrum", n, xmn, xmx);
  // 6
  analysisMan->CreateH1("WLS time", "WLS emission time", n, xmn, xmx);
  // 7
  analysisMan->CreateH1("WLS2 abs", "WLS2 absorption spectrum", n, xmn, xmx);
  // 8
  analysisMan->CreateH1("WLS2 em", "WLS2 emission spectrum", n, xmn, xmx);
  // 9
  analysisMan->CreateH1("WLS2 time", "WLS2 emission time", n, xmn, xmx);
  // 10
  analysisMan->CreateH1("bdry status", "boundary process status", n, xmn, xmx);
  // 11
  analysisMan->CreateH1(
    "x_backward", "X momentum dir of backward-going photons", n, xmn, xmx);
  // 12
  analysisMan->CreateH1(
    "y_backward", "Y momentum dir of backward-going photons", n, xmn, xmx);
  // 13
  analysisMan->CreateH1(
    "z_backward", "Z momentum dir of backward-going photons", n, xmn, xmx);
  // 14
  analysisMan->CreateH1("x_forward", "X momentum dir of forward-going photons",
                        n, xmn, xmx);
  // 15
  analysisMan->CreateH1("y_forward", "Y momentum dir of forward-going photons",
                        n, xmn, xmx);
  // 16
  analysisMan->CreateH1("z_forward", "Z momentum dir of forward-going photons",
                        n, xmn, xmx);
  // 17
  analysisMan->CreateH1(
    "x_fresnel", "X momentum dir of Fresnel-refracted photons", n, xmn, xmx);
  // 18
  analysisMan->CreateH1(
    "y_fresnel", "Y momentum dir of Fresnel-refracted photons", n, xmn, xmx);
  // 19
  analysisMan->CreateH1(
    "z_fresnel", "Z momentum dir of Fresnel-refracted photons", n, xmn, xmx);
  // 20
  analysisMan->CreateH1("Transmitted", "Transmitted photons", n, xmn, xmx);
  // 21
  analysisMan->CreateH1("Reflected", "Reflected photons", n, xmn, xmx);
  //22
  analysisMan->CreateH1("All", "All photons", n, xmn, xmx);
  //23
  analysisMan->CreateH1("Trans_azim", "Transmitted photons", n, xmn, xmx);
  //24
  analysisMan->CreateH1("Refl_azim", "Reflected photons", n, xmn, xmx);
  //25
  analysisMan->CreateH1("All_azim", "All photons", n, xmn, xmx);
  //26
  analysisMan->CreateH1("Time", "Time of Arrival", n, xmn, xmx);
  //27
  analysisMan->CreateH1("sLength", "Step Length", n, xmn, xmx);
  //28
  analysisMan->CreateH1("sNum", "Step Number", n, xmn, xmx);
  //29
  analysisMan->CreateH1("Polar 2nd peak", "Polar 2nd peak", n, xmn, xmx);
  //30
  analysisMan->CreateH1("Azim 2nd peak", "Azim 2nd peak", n, xmn, xmx);
  //31
  analysisMan->CreateH1("Reflected_NF", "Reflected photons", n, xmn, xmx);
  //32
  analysisMan->CreateH1("Transmitted_NF", "Transmitted photons", n, xmn, xmx);

  //33
  analysisMan->CreateH1("momi_x", "momi_x", n, xmn, xmx);
  //34
  analysisMan->CreateH1("momi_y", "momi_y", n, xmn, xmx);
  //35
  analysisMan->CreateH1("momi_z", "momi_z", n, xmn, xmx);

  //36
  analysisMan->CreateH1("momb_x", "momb_x", n, xmn, xmx);
  //37
  analysisMan->CreateH1("momb_y", "momb_y", n, xmn, xmx);
  //38
  analysisMan->CreateH1("momb_z", "momb_z", n, xmn, xmx);

  //39
  analysisMan->CreateH1("mompmt_x", "mompmt_x", n, xmn, xmx);
  //40
  analysisMan->CreateH1("mompmt_y", "mompmt_y", n, xmn, xmx);
  //41
  analysisMan->CreateH1("mompmt_z", "mompmt_z", n, xmn, xmx);



  for(G4int i = 0; i < analysisMan->GetNofH1s(); ++i)
  {
    analysisMan->SetH1Activation(i, false);
  }
  
  analysisMan->SetFirstNtupleId(0);
  analysisMan->SetNtupleMerging( 1 ); 

  analysisMan->CreateNtuple("momi", "momi");
  analysisMan->CreateNtupleDColumn("x");
  analysisMan->CreateNtupleDColumn("y");
  analysisMan->CreateNtupleDColumn("z");
  analysisMan->FinishNtuple(); 

  analysisMan->CreateNtuple("momb", "momb");
  analysisMan->CreateNtupleDColumn("x");
  analysisMan->CreateNtupleDColumn("y");
  analysisMan->CreateNtupleDColumn("z");
  analysisMan->FinishNtuple(); 

  analysisMan->CreateNtuple("mompmt", "mompmt");
  analysisMan->CreateNtupleDColumn("x");
  analysisMan->CreateNtupleDColumn("y");
  analysisMan->CreateNtupleDColumn("z");
  analysisMan->FinishNtuple(); 

  analysisMan->CreateNtuple("mombi", "mombi");
  analysisMan->CreateNtupleDColumn("x");
  analysisMan->CreateNtupleDColumn("y");
  analysisMan->CreateNtupleDColumn("z");
  analysisMan->FinishNtuple(); 

  analysisMan->CreateNtuple("mompmti", "mompmti");
  analysisMan->CreateNtupleDColumn("x");
  analysisMan->CreateNtupleDColumn("y");
  analysisMan->CreateNtupleDColumn("z");
  analysisMan->FinishNtuple(); 

  analysisMan->CreateNtuple("pos_esc", "pos_esc");
  analysisMan->CreateNtupleDColumn("x");
  analysisMan->CreateNtupleDColumn("y");
  analysisMan->CreateNtupleDColumn("z");
  analysisMan->FinishNtuple(); 

  analysisMan->CreateNtuple("bound", "bound");
  analysisMan->CreateNtupleIColumn("code");
  analysisMan->FinishNtuple(); 

}
