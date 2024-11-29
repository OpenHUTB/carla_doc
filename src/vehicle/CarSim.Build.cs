using System.IO;
using System;

namespace UnrealBuildTool.Rules
{
    public class CarSim : ModuleRules
    {
        private string BaseDir
        {
            get { return Path.GetFullPath(Path.Combine(ModuleDirectory, "../..")); }
        }

        private string ThirdPartyDir
        {
            get { return Path.Combine(BaseDir, "ThirdParty"); }
        }

        private bool IsDebug(ReadOnlyTargetRules Target)
        {
            return Target.Configuration == UnrealTargetConfiguration.Debug
                    || Target.Configuration == UnrealTargetConfiguration.DebugGame;
        }

        private bool IsWindows(ReadOnlyTargetRules Target)
        {
            return Target.Platform == UnrealTargetPlatform.Win64
                    || Target.Platform == UnrealTargetPlatform.Win32;
        }

        public CarSim(ReadOnlyTargetRules Target)
        : base(Target)
        {
            PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
            PrivatePCHHeaderFile = "Public/CarSimPCH.h";

            string WrapperBaseDirectory = Path.Combine(ThirdPartyDir, "vs_vehicle");
            string WrapperBuildDirectory = Path.Combine(WrapperBaseDirectory, "build");
            string WrapperPlatform;
            string WrapperConfig;
            string WrapperDllName;
            string SolversDirectory;

            if (Target.Platform == UnrealTargetPlatform.Win32)
            {
                WrapperPlatform = "Win32";

                // CarSim 32-bit solvers:
                SolversDirectory = Path.Combine(ThirdPartyDir, "VehicleSim/Prog");
            }
            else if (Target.Platform == UnrealTargetPlatform.Win64)
            {
                WrapperPlatform = "x64";

                // CarSim 64-bit solvers:
                SolversDirectory = Path.Combine(ThirdPartyDir, "VehicleSim/Prog");
            }
            else
            {
                WrapperPlatform = "Linux";
                SolversDirectory = Path.Combine(ThirdPartyDir, "VehicleSim/Prog");
            }

            WrapperConfig = "Release";
            WrapperDllName = "libvs_vehicle.so.2021.0";

            string VsVehicleWrapperPath = Path.Combine(WrapperBuildDirectory, WrapperPlatform, WrapperConfig);
            string VsVehicleWrapperDll = Path.Combine(VsVehicleWrapperPath, WrapperDllName);

            PrivateIncludePaths.AddRange(
                new string[] {  "CarSim/Private", WrapperBaseDirectory
                }
            );

            // Names of modules that this module's public interface depends on.
            PublicDependencyModuleNames.AddRange(
                new string[]
                {
                  "Core"
                , "CoreUObject"
                , "Engine"
                }
            );

            if (IsWindows(Target))
            {
                // Windows-only dependencies:
                PublicDependencyModuleNames.AddRange(
                    new string[]
                    {
                    "DisplayCluster"
                    }
                );
            }

            // Names of modules that this module's private implementation depends on.
            PrivateDependencyModuleNames.AddRange(
                new string[] {
                "Projects"
                }
            );

            // What I [bdp] think this does: Tells build system that this file is
            // required at runtime, so the build system will package it when the
            // shipping "game" is built ("packaged").
            string VsConnectDll = Path.Combine(ThirdPartyDir, "vs_connect/libvs_connect.so.2021.0");

            RuntimeDependencies.Add(VsConnectDll);
            RuntimeDependencies.Add(VsVehicleWrapperDll);

            /// \todo Iterate over all DLL's in CarSimSolversDirectory and add them all to RuntimeDependencies() instead of specifying hard-coded filename(s):
            RuntimeDependencies.Add(Path.Combine(SolversDirectory, "libcarsim.so.2021.0"));
            RuntimeDependencies.Add(Path.Combine(SolversDirectory, "libtrucksim.so.2021.0"));

            // CarSim Parsfiles:
            string RunsDirectory = Path.Combine(ThirdPartyDir, "VehicleSim/Data");
            /// \todo Iterate over all parsfiles in CarSimRunsDirectory and add them all to RuntimeDependencies().
            RuntimeDependencies.Add(Path.Combine(RunsDirectory, "carsim_all.par"));
            RuntimeDependencies.Add(Path.Combine(RunsDirectory, "trucksim_all.par"));

            // List of delay load DLLs - typically used for External (third party) modules
            PublicDelayLoadDLLs.Add(VsVehicleWrapperDll);

            // VS Connect:
            PublicDelayLoadDLLs.Add(VsConnectDll);
        }
    }
}