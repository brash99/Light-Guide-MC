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
//
/// \file optical/OpNovice2/src/SteppingAction.cc
/// \brief Implementation of the SteppingAction class
//
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

#include "SteppingAction.hh"
#include "DetectorConstruction.hh"
#include "EventAction.hh"

#include "HistoManager.hh"
#include "Run.hh"
#include "TrackInformation.hh"

#include "G4Cerenkov.hh"
#include "G4Event.hh"
#include "G4EventManager.hh"
#include "G4Scintillation.hh"
#include "G4OpBoundaryProcess.hh"
#include "G4OpticalPhoton.hh"
#include "G4ProcessManager.hh"
#include "G4Step.hh"
#include "G4SteppingManager.hh"
#include "G4SystemOfUnits.hh"
#include "G4Track.hh"
#include "G4TrackList.hh"
#include "G4RunManager.hh"
#include "G4LogicalVolume.hh"
#include "G4VPhysicalVolume.hh"
#include "G4AccumulableManager.hh"

#include "G4VisManager.hh"
#include "G4Trajectory.hh"


//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
SteppingAction::SteppingAction(EventAction* eventAction)
  : G4UserSteppingAction()
  , fVerbose(0)
  , fEventAction(eventAction)
{}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
SteppingAction::~SteppingAction() {}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void SteppingAction::UserSteppingAction(const G4Step* step)
{
  
  static G4ParticleDefinition* opticalphoton =
    G4OpticalPhoton::OpticalPhotonDefinition();

  G4AnalysisManager* analysisMan = G4AnalysisManager::Instance();
  Run* run =
    static_cast<Run*>(G4RunManager::GetRunManager()->GetNonConstCurrentRun());

  G4Track* track          = step->GetTrack();
  G4StepPoint* endPoint   = step->GetPostStepPoint();
  G4StepPoint* startPoint = step->GetPreStepPoint();
  

  //new

  G4ThreeVector start_pos = startPoint->GetPosition();

  auto PreVolume = step->GetPreStepPoint()->GetPhysicalVolume();
  auto PostVolume = step->GetPostStepPoint()->GetPhysicalVolume();

  const DetectorConstruction* detConstruction
      = static_cast<const DetectorConstruction*>
        (G4RunManager::GetRunManager()->GetUserDetectorConstruction());
  auto detVolume = detConstruction->GetScoringVolume();
  auto detVolume_mid = detConstruction->GetMidVolume();
  auto scint_volume = detConstruction->GetScintVolume();
  auto tank = detConstruction->GetTankVolume();
  //G4bool condition = step->IsFirstStepInVolume();

  G4int counter_step = 0;
  G4double time = 0;
  G4double sLength = 0;
  G4double stepNum = 0;
  G4int time_min = 3.0;
  G4int time_max = 5.0;
  G4bool condition = false;

  G4double angle_time = startPoint->GetMomentumDirection().getTheta();
  G4double azim_time = startPoint->GetMomentumDirection().getPhi();
  G4bool in = false;
  G4ThreeVector mom = startPoint->GetMomentumDirection();
  G4ThreeVector initialMomentum = track->GetVertexMomentumDirection();
  G4double initialTheta = initialMomentum.theta();
   if (initialTheta > CLHEP::pi - 0.7297 && initialTheta < CLHEP::pi + 0.7297) {
      in = true;
   }

  TrackInformation* trackInfo = (TrackInformation*) (track->GetUserInformation());
  const G4DynamicParticle* theParticle = track->GetDynamicParticle();
  const G4ParticleDefinition* particleDef = theParticle->GetParticleDefinition();

  if (PostVolume != detVolume_mid && PostVolume != detVolume && PostVolume != scint_volume && PreVolume != detVolume_mid && PreVolume != detVolume && PreVolume != scint_volume){
    analysisMan->FillNtupleDColumn(0, 0, initialMomentum[0]); 
    analysisMan->FillNtupleDColumn(0, 1, initialMomentum[1]); 
    analysisMan->FillNtupleDColumn(0, 2, initialMomentum[2]); 
    analysisMan->AddNtupleRow(0);
    analysisMan->FillH1(33, initialMomentum[0]);
    analysisMan->FillH1(34, initialMomentum[1]);
    analysisMan->FillH1(35, initialMomentum[2]);
    track->SetTrackStatus(fStopAndKill);
  }

 if(PostVolume == tank)
  {
    track->SetTrackStatus(fStopAndKill);
  }

  if(PreVolume == detVolume_mid && PostVolume == detVolume)
  {
    analysisMan->FillNtupleDColumn(2, 0, mom[0]); 
    analysisMan->FillNtupleDColumn(2, 1, mom[1]); 
    analysisMan->FillNtupleDColumn(2, 2, mom[2]); 
    analysisMan->AddNtupleRow(2);
    analysisMan->FillNtupleDColumn(4, 0, initialMomentum[0]); 
    analysisMan->FillNtupleDColumn(4, 1, initialMomentum[1]); 
    analysisMan->FillNtupleDColumn(4, 2, initialMomentum[2]); 
    analysisMan->AddNtupleRow(4);
    analysisMan->FillH1(39, mom[0]);
    analysisMan->FillH1(40, mom[1]);
    analysisMan->FillH1(41, mom[2]);
    counter_step += 1;
    //G4cout << "PreVolume == detVolume_mid && PostVolume == detVolume:  " << counter_step << G4endl;
    // time = step->GetPreStepPoint()->GetGlobalTime();

    // if(time_min <= (time * ns) && (time * ns) <= time_max)
    // {
    //   analysisMan->FillH1(29, angle_time / deg);
    //   analysisMan->FillH1(30, azim_time / deg);
    // }

    //sLength = step->GetStepLength();
    //analysisMan->FillH1(27, sLength / m);
    //stepNum = track->GetCurrentStepNumber();
    //analysisMan->FillH1(28, stepNum);
    // G4VVisManager* visManager = G4VVisManager::GetConcreteInstance();
    // if (visManager) {
    //     G4Trajectory* trajectory = new G4Trajectory(track);
    //     trajectory->ShowTrajectory();
    // }
    G4int boIn = trackInfo->getBouncesIn();
    G4int boOut = trackInfo->getBouncesOut();
    run->addBounceIn(boIn);
    run->addBounceOut(boOut);
    if (in){
      run->addParticleIn();
      analysisMan->FillH1(26, time * ns);
    }
    else {
      run->addParticleOut();
      analysisMan->FillH1(27, time * ns);
    }
    analysisMan->FillNtupleDColumn(0, 0, initialMomentum[0]); 
    analysisMan->FillNtupleDColumn(0, 1, initialMomentum[1]); 
    analysisMan->FillNtupleDColumn(0, 2, initialMomentum[2]); 
    analysisMan->AddNtupleRow(0);
    analysisMan->FillH1(33, initialMomentum[0]);
    analysisMan->FillH1(34, initialMomentum[1]);
    analysisMan->FillH1(35, initialMomentum[2]);
    track->SetTrackStatus(fStopAndKill);
  }
  fEventAction->AddCount(counter_step);
  if (!in) {
      fEventAction->AddCount_PMT_NT(counter_step);
  }
  
  G4int counter_step4 = 0;
  if(PreVolume == scint_volume && PostVolume == detVolume_mid)
  {
    analysisMan->FillNtupleDColumn(1, 0, mom[0]); 
    analysisMan->FillNtupleDColumn(1, 1, mom[1]); 
    analysisMan->FillNtupleDColumn(1, 2, mom[2]); 
    analysisMan->AddNtupleRow(1);
    analysisMan->FillNtupleDColumn(3, 0, initialMomentum[0]); 
    analysisMan->FillNtupleDColumn(3, 1, initialMomentum[1]); 
    analysisMan->FillNtupleDColumn(3, 2, initialMomentum[2]); 
    analysisMan->AddNtupleRow(3);
    analysisMan->FillH1(36, mom[0]);
    analysisMan->FillH1(37, mom[1]);
    analysisMan->FillH1(38, mom[2]);
    counter_step4 += 1;
    //G4cout << "PreVolume == scint_volume && PostVolume == detVolume_mid:  " << counter_step4 << G4endl;
  }
  fEventAction->AddCount_mid(counter_step4);
  if (!in) {
      fEventAction->AddCount_LG_NT(counter_step4);
  }

  if(PostVolume == scint_volume && PreVolume == detVolume_mid)
  {
    analysisMan->FillNtupleDColumn(0, 0, initialMomentum[0]); 
    analysisMan->FillNtupleDColumn(0, 1, initialMomentum[1]); 
    analysisMan->FillNtupleDColumn(0, 2, initialMomentum[2]); 
    analysisMan->AddNtupleRow(0);
  //   analysisMan->FillH1(33, initialMomentum[0]);
  //   analysisMan->FillH1(34, initialMomentum[1]);
  //   analysisMan->FillH1(35, initialMomentum[2]);
    track->SetTrackStatus(fStopAndKill);
  }


  if(particleDef == opticalphoton)
  {
    const G4VProcess* pds = endPoint->GetProcessDefinedStep();
    G4String procname     = pds->GetProcessName();

    if(procname.compare("OpAbsorption") == 0)
    {
      run->AddOpAbsorption();
      if(trackInfo->GetIsFirstTankX())
      {
        run->AddOpAbsorptionPrior();
      }
    }
    else if(procname.compare("OpRayleigh") == 0)
    {
      run->AddRayleigh();
    }
    else if(procname.compare("OpWLS") == 0)
    {
      G4double en = track->GetKineticEnergy();
      run->AddWLSAbsorption();
      run->AddWLSAbsorptionEnergy(en);
      analysisMan->FillH1(4, en / eV);  // absorption energy
      // loop over secondaries, create statistics
      // const std::vector<const G4Track*>* secondaries =
      auto secondaries = step->GetSecondaryInCurrentStep();
      for(auto sec : *secondaries)
      {
        en = sec->GetKineticEnergy();
        run->AddWLSEmission();
        run->AddWLSEmissionEnergy(en);
        analysisMan->FillH1(5, en / eV);  // emission energy
        G4double time = sec->GetGlobalTime();
        analysisMan->FillH1(6, time / ns);
      }
    }
    else if(procname.compare("OpWLS2") == 0)
    {
      G4double en = track->GetKineticEnergy();
      run->AddWLS2Absorption();
      run->AddWLS2AbsorptionEnergy(en);
      analysisMan->FillH1(7, en / eV);  // absorption energy
      // loop over secondaries, create statistics
      // const std::vector<const G4Track*>* secondaries =
      auto secondaries = step->GetSecondaryInCurrentStep();
      for(auto sec : *secondaries)
      {
        en = sec->GetKineticEnergy();
        run->AddWLS2Emission();
        run->AddWLS2EmissionEnergy(en);
        analysisMan->FillH1(8, en / eV);  // emission energy
        G4double time = sec->GetGlobalTime();
        analysisMan->FillH1(9, time / ns);
      }
    }

    // optical process has endpt on bdry,
    if(endPoint->GetStepStatus() == fGeomBoundary)
    {
      G4ThreeVector m0 = startPoint->GetMomentumDirection();
      G4ThreeVector m1 = endPoint->GetMomentumDirection();

      G4OpBoundaryProcessStatus theStatus = Undefined;

      G4ProcessManager* OpManager = opticalphoton->GetProcessManager();
      G4ProcessVector* postStepDoItVector =
        OpManager->GetPostStepProcessVector(typeDoIt);
      G4int n_proc = postStepDoItVector->entries();

      if (true) //(trackInfo->GetIsFirstTankX())
      if (PostVolume == tank)
      {
        
      }
      {
        G4ThreeVector momdir = endPoint->GetMomentumDirection();
        G4double px1         = momdir.x();
        G4double py1         = momdir.y();
        G4double pz1         = momdir.z();
        if(px1 < 0.)
        {
          analysisMan->FillH1(11, px1);
          analysisMan->FillH1(12, py1);
          analysisMan->FillH1(13, pz1);
        }
        else
        {
          analysisMan->FillH1(14, px1);
          analysisMan->FillH1(15, py1);
          analysisMan->FillH1(16, pz1);
        }

        trackInfo->SetIsFirstTankX(false);
        run->AddTotalSurface();
        
        for(G4int i = 0; i < n_proc; ++i)
        {
          G4VProcess* currentProcess = (*postStepDoItVector)[i];

          G4OpBoundaryProcess* opProc =
            dynamic_cast<G4OpBoundaryProcess*>(currentProcess);
          if(opProc)
          {
            G4ThreeVector mom_dir = startPoint->GetMomentumDirection();
            G4double mom_x = mom_dir.x();
            G4double mom_y = mom_dir.y();
            G4double mom_z = mom_dir.z();
            //G4double angle = std::acos(mom_z);
            G4double angle = startPoint->GetMomentumDirection().getTheta();
            G4double azim = startPoint->GetMomentumDirection().getPhi();


            //G4cout << " angle/azim = " << angle / deg << "/" << azim / deg << " deg" << G4endl;
            //G4double azim2 = std::acos(mom_x);

            theStatus      = opProc->GetStatus();
            analysisMan->FillH1(10, theStatus);
            analysisMan->FillNtupleIColumn(6, 0, theStatus);
            analysisMan->AddNtupleRow(6);
            G4ThreeVector trkPos = track->GetPosition();

            switch(theStatus)
            {
              case Transmission:
                run->AddTransmission();
                break;
              case FresnelRefraction:
                if (true)//(PreVolume == detVolume_mid)// && (PostVolume != detVolume && PostVolume != detVolume_mid && PostVolume != scint_volume))
                {
                  analysisMan->FillNtupleDColumn(5, 0, start_pos.x()); 
                  analysisMan->FillNtupleDColumn(5, 1, start_pos.y()); 
                  analysisMan->FillNtupleDColumn(5, 2, start_pos.z()); 
                  analysisMan->AddNtupleRow(5);
                }
                run->AddFresnelRefraction();
                analysisMan->FillH1(17, px1);
                analysisMan->FillH1(18, py1);
                analysisMan->FillH1(19, pz1);

                // transmission
                analysisMan->FillH1(20, angle / deg);
                analysisMan->FillH1(22, angle / deg);
                analysisMan->FillH1(23, azim / deg);
                analysisMan->FillH1(25, azim / deg);
                analysisMan->FillH1(32, angle / deg);
                if (in) {
                  trackInfo->addBounceIn();
                }
                else {
                  trackInfo->addBounceOut();
                }
                track->SetTrackStatus(fStopAndKill);
                break;
              case FresnelReflection:
                run->AddFresnelReflection();
                analysisMan->FillH1(21, angle / deg);
                analysisMan->FillH1(22, angle / deg);
                analysisMan->FillH1(24, azim / deg);
                analysisMan->FillH1(25, azim / deg);
                analysisMan->FillH1(31, angle / deg);
                if (in) {
                  trackInfo->addBounceIn();
                }
                else {
                  trackInfo->addBounceOut();
                }
                break;
              case TotalInternalReflection:
                run->AddTotalInternalReflection();
                analysisMan->FillH1(21, angle / deg);
                analysisMan->FillH1(22, angle / deg);
                analysisMan->FillH1(24, azim / deg);
                analysisMan->FillH1(25, azim / deg);
                analysisMan->FillH1(31, angle / deg);
                if (in) {
                  trackInfo->addBounceIn();
                }
                else {
                  trackInfo->addBounceOut();
                }
                break;
              case LambertianReflection:
                run->AddLambertianReflection();
                if (in) {
                  trackInfo->addBounceIn();
                }
                else {
                  trackInfo->addBounceOut();
                }
                break;
              case LobeReflection:
                run->AddLobeReflection();
                if (in) {
                  trackInfo->addBounceIn();
                }
                else {
                  trackInfo->addBounceOut();
                }
                break;
              case SpikeReflection:
                run->AddSpikeReflection();
                if (in) {
                  trackInfo->addBounceIn();
                }
                else {
                  trackInfo->addBounceOut();
                }
                break;
              case BackScattering:
                run->AddBackScattering();
                if (in) {
                  trackInfo->addBounceIn();
                }
                else {
                  trackInfo->addBounceOut();
                }
                break;
              case Absorption:
                run->AddAbsorption();
                break;
              case Detection:
                run->AddDetection();
                break;
              case NotAtBoundary:
                run->AddNotAtBoundary();
                break;
              case SameMaterial:
                run->AddSameMaterial();
                break;
              case StepTooSmall:
                run->AddStepTooSmall();
                break;
              case NoRINDEX:
                run->AddNoRINDEX();
                break;
              case PolishedLumirrorAirReflection:
                run->AddPolishedLumirrorAirReflection();
                break;
              case PolishedLumirrorGlueReflection:
                run->AddPolishedLumirrorGlueReflection();
                break;
              case PolishedAirReflection:
                run->AddPolishedAirReflection();
                break;
              case PolishedTeflonAirReflection:
                run->AddPolishedTeflonAirReflection();
                break;
              case PolishedTiOAirReflection:
                run->AddPolishedTiOAirReflection();
                break;
              case PolishedTyvekAirReflection:
                run->AddPolishedTyvekAirReflection();
                break;
              case PolishedVM2000AirReflection:
                run->AddPolishedVM2000AirReflection();
                break;
              case PolishedVM2000GlueReflection:
                run->AddPolishedVM2000AirReflection();
                break;
              case EtchedLumirrorAirReflection:
                run->AddEtchedLumirrorAirReflection();
                break;
              case EtchedLumirrorGlueReflection:
                run->AddEtchedLumirrorGlueReflection();
                break;
              case EtchedAirReflection:
                run->AddEtchedAirReflection();
                break;
              case EtchedTeflonAirReflection:
                run->AddEtchedTeflonAirReflection();
                break;
              case EtchedTiOAirReflection:
                run->AddEtchedTiOAirReflection();
                break;
              case EtchedTyvekAirReflection:
                run->AddEtchedTyvekAirReflection();
                break;
              case EtchedVM2000AirReflection:
                run->AddEtchedVM2000AirReflection();
                break;
              case EtchedVM2000GlueReflection:
                run->AddEtchedVM2000AirReflection();
                break;
              case GroundLumirrorAirReflection:
                run->AddGroundLumirrorAirReflection();
                break;
              case GroundLumirrorGlueReflection:
                run->AddGroundLumirrorGlueReflection();
                break;
              case GroundAirReflection:
                run->AddGroundAirReflection();
                break;
              case GroundTeflonAirReflection:
                run->AddGroundTeflonAirReflection();
                break;
              case GroundTiOAirReflection:
                run->AddGroundTiOAirReflection();
                break;
              case GroundTyvekAirReflection:
                run->AddGroundTyvekAirReflection();
                break;
              case GroundVM2000AirReflection:
                run->AddGroundVM2000AirReflection();
                break;
              case GroundVM2000GlueReflection:
                run->AddGroundVM2000AirReflection();
                break;
              case Dichroic:
                run->AddDichroic();
                break;
              default:
                G4cout << "theStatus: " << theStatus
                       << " was none of the above." << G4endl;
                break;
            }
          }
        }
      }
    }
  }

  else
  {  // particle != opticalphoton
    // print how many Cerenkov and scint photons produced this step
    // this demonstrates use of GetNumPhotons()
    auto proc_man =
      track->GetDynamicParticle()->GetParticleDefinition()->GetProcessManager();
    G4ProcessVector* proc_vec = proc_man->GetPostStepProcessVector(typeDoIt);
    G4int n_proc              = proc_vec->entries();

    G4int n_scint = 0;
    G4int n_cer   = 0;
    for(G4int i = 0; i < n_proc; ++i)
    {
      G4String proc_name = (*proc_vec)[i]->GetProcessName();
      if(proc_name.compare("Cerenkov") == 0)
      {
        auto cer = (G4Cerenkov*) (*proc_vec)[i];
        n_cer    = cer->GetNumPhotons();
      }
      else if(proc_name.compare("Scintillation") == 0)
      {
        auto scint = (G4Scintillation*) (*proc_vec)[i];
        n_scint    = scint->GetNumPhotons();
      }
    }
    if(fVerbose > 0)
    {
      if(n_cer > 0 || n_scint > 0)
      {
        G4cout << "In this step, " << n_cer << " Cerenkov and " << n_scint
               << " scintillation photons were produced." << G4endl;
      }
    }

    // loop over secondaries, create statistics
    const std::vector<const G4Track*>* secondaries =
      step->GetSecondaryInCurrentStep();

    for(auto sec : *secondaries)
    {
      if(sec->GetDynamicParticle()->GetParticleDefinition() == opticalphoton)
      {
        G4String creator_process = sec->GetCreatorProcess()->GetProcessName();
        if(creator_process.compare("Cerenkov") == 0)
        {
          G4double en = sec->GetKineticEnergy();
          run->AddCerenkovEnergy(en);
          run->AddCerenkov();
          analysisMan->FillH1(1, en / eV);
        }
        else if(creator_process.compare("Scintillation") == 0)
        {
          G4double en = sec->GetKineticEnergy();
          run->AddScintillationEnergy(en);
          run->AddScintillation();
          analysisMan->FillH1(2, en / eV);

          G4double time = sec->GetGlobalTime();
          analysisMan->FillH1(3, time / ns);
        }
      }
    }
  }

  return;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
