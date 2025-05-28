package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * This OpMode implements a Genetic Algorithm (GA) for auto-tuning PID values
 * for translational, heading, and a general "drive" component within the
 * Pedro Pathing framework.
 *
 * The GA works by evolving a population of PID parameter sets. Each set
 * (individual) is evaluated by running the robot on a test path, and a
 * "fitness" score is assigned based on performance. Fitter individuals are
 * selected to create the next generation through crossover and mutation.
 *
 * IMPORTANT: The `calculateFitness` method is a conceptual placeholder.
 * You MUST implement this method to measure actual robot performance
 * (e.g., error, overshoot, time) and return a fitness score.
 *
 * @author Your Team Name
 * @version 1.0, 5/28/2025
 */
@Config // Allows FtcDashboard to configure GA parameters and display best PID values
@Autonomous(name = "GA PID Auto Tuner", group = "PIDF Testing")
public class AutoPIDTunerGA extends OpMode {

    private Telemetry telemetryA;
    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();
    private Random random = new Random();

    // --- Genetic Algorithm Parameters (Configurable via FtcDashboard) ---
    public static int POPULATION_SIZE = 20;
    public static int MAX_GENERATIONS = 50;
    public static double MUTATION_RATE = 0.1; // Probability of a gene mutating (0.0 to 1.0)
    public static double MUTATION_STRENGTH = 0.1; // How much a gene can change during mutation
    public static double PID_MIN_VALUE = 0.0; // Minimum possible value for any PID gain
    public static double PID_MAX_VALUE = 2.0; // Maximum possible value for any PID gain

    // --- Robot Test Path Parameters ---
    public static double TEST_DISTANCE = 30; // Distance for the robot to travel during tuning

    // --- GA State Variables ---
    private enum TuningState {
        INITIALIZING,
        EVALUATING_INDIVIDUAL,
        EVOLVING_POPULATION,
        FINISHED
    }
    private TuningState currentState = TuningState.INITIALIZING;

    private List<Individual> population;
    private Individual bestIndividualEver;
    private int currentGeneration;
    private int currentIndividualIndex;

    private Path testPath; // The path used to evaluate each PID set

    /**
     * Represents a single set of PID parameters (a "chromosome") and its fitness.
     * Genes are ordered: [P_TRANS, I_TRANS, D_TRANS, P_HEAD, I_HEAD, D_HEAD, P_DRIVE, I_DRIVE, D_DRIVE]
     */
    private class Individual {
        public double[] pidParams; // Array to hold P, I, D for translational, heading, and drive
        public double fitness;     // Lower fitness is better

        /**
         * Constructor for a new individual with random PID parameters.
         * There are 9 PID parameters in total (3 for translational, 3 for heading, 3 for drive).
         */
        public Individual() {
            pidParams = new double[9];
            for (int i = 0; i < pidParams.length; i++) {
                pidParams[i] = random.nextDouble() * (PID_MAX_VALUE - PID_MIN_VALUE) + PID_MIN_VALUE;
            }
            fitness = Double.POSITIVE_INFINITY; // Initialize with infinite fitness
        }

        /**
         * Constructor for creating an individual with specific PID parameters.
         * @param params An array of 9 doubles representing the PID gains.
         */
        public Individual(double[] params) {
            this.pidParams = params;
            this.fitness = Double.POSITIVE_INFINITY;
        }

        /**
         * Creates a deep copy of this individual.
         * @return A new Individual object with the same PID parameters and fitness.
         */
        public Individual clone() {
            Individual newIndividual = new Individual(pidParams.clone());
            newIndividual.fitness = this.fitness;
            return newIndividual;
        }

        /**
         * Returns a string representation of the PID parameters.
         */
        @Override
        public String toString() {
            return String.format(
                    "P_T: %.3f, I_T: %.3f, D_T: %.3f | P_H: %.3f, I_H: %.3f, D_H: %.3f | P_D: %.3f, I_D: %.3f, D_D: %.3f (Fitness: %.4f)",
                    pidParams[0], pidParams[1], pidParams[2], // Translational
                    pidParams[3], pidParams[4], pidParams[5], // Heading
                    pidParams[6], pidParams[7], pidParams[8], // Drive
                    fitness
            );
        }
    }

    /**
     * This initializes the Follower, sets up telemetry, and prepares the
     * initial population for the Genetic Algorithm.
     */
    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(hardwareMap, AConstants.class, LConstants.class);

        // Define the test path for evaluating individuals (a simple straight line)
        testPath = new Path(new BezierCurve(new Point(0, 0, Point.CARTESIAN), new Point(TEST_DISTANCE, 0, Point.CARTESIAN)));

        telemetryA.addLine("GA PID Auto Tuner Initialized.");
        telemetryA.addLine("Ensure robot has clear space for " + TEST_DISTANCE + " inches.");
        telemetryA.addLine("Starting Genetic Algorithm...");
        telemetryA.update();

        initializePopulation();
        currentGeneration = 0;
        currentIndividualIndex = 0;
        bestIndividualEver = null;
        currentState = TuningState.EVALUATING_INDIVIDUAL; // Start evaluation immediately
        runtime.reset();
    }

    /**
     * The main loop of the OpMode, driving the Genetic Algorithm.
     * It manages the state transitions between evaluating individuals and evolving the population.
     */
    @Override
    public void loop() {
        // Always update the follower, even if not actively following a path,
        // as it processes sensor data needed for fitness evaluation.
        follower.update();

        switch (currentState) {
            case INITIALIZING:
                // This state is handled in init()
                break;

            case EVALUATING_INDIVIDUAL:
                // Check if there are more individuals to evaluate in the current generation
                if (currentIndividualIndex < population.size()) {
                    Individual currentIndividual = population.get(currentIndividualIndex);

                    // Apply the current individual's PID parameters to AConstants
                    applyPIDToAConstants(currentIndividual);

                    // Command the robot to follow the test path
                    // This assumes follower.isBusy() will become true and then false when path completes
                    if (!follower.isBusy()) {
                        // If follower just finished a path, calculate fitness for the previous individual
                        if (currentIndividualIndex > 0) { // Don't calculate for the very first individual before it runs
                            Individual evaluatedIndividual = population.get(currentIndividualIndex - 1);
                            evaluatedIndividual.fitness = calculateFitness(evaluatedIndividual); // THIS IS THE CRITICAL CALL
                            updateBestIndividual(evaluatedIndividual);
                        }
                        // Now start the next individual's run
                        follower.followPath(testPath);
                        runtime.reset(); // Reset runtime to track individual's run time
                        telemetryA.addData("Evaluating Individual", (currentIndividualIndex + 1) + "/" + population.size());
                        telemetryA.addData("Current PID Set", currentIndividual.toString());
                        currentIndividualIndex++; // Move to the next individual
                    } else {
                        // Robot is currently running the path, just wait
                        telemetryA.addData("Running Path", "Time: %.2f s", runtime.seconds());
                    }
                } else {
                    // All individuals in the current generation have been evaluated
                    // Calculate fitness for the very last individual after its path completes
                    if (!follower.isBusy() && currentIndividualIndex == population.size()) {
                        Individual lastIndividual = population.get(population.size() - 1);
                        lastIndividual.fitness = calculateFitness(lastIndividual);
                        updateBestIndividual(lastIndividual);
                        currentIndividualIndex++; // Increment to mark all evaluated
                    }

                    if (currentIndividualIndex > population.size()) { // All evaluated and last fitness calculated
                        currentState = TuningState.EVOLVING_POPULATION;
                        telemetryA.addLine("Generation " + currentGeneration + " evaluation complete.");
                        telemetryA.addData("Best Fitness This Gen", getBestFitnessInCurrentGeneration());
                        telemetryA.addData("Overall Best", bestIndividualEver.toString());
                    }
                }
                break;

            case EVOLVING_POPULATION:
                if (currentGeneration < MAX_GENERATIONS) {
                    evolvePopulation();
                    currentGeneration++;
                    currentIndividualIndex = 0; // Reset index for the new generation
                    currentState = TuningState.EVALUATING_INDIVIDUAL;
                    telemetryA.addLine("Starting Generation " + currentGeneration + "...");
                } else {
                    currentState = TuningState.FINISHED;
                    telemetryA.addLine("Max Generations Reached. Tuning Complete!");
                }
                break;

            case FINISHED:
                telemetryA.addLine("GA Tuning Finished!");
                if (bestIndividualEver != null) {
                    telemetryA.addLine("Overall Best PID Parameters:");
                    telemetryA.addData("Best PID Set", bestIndividualEver.toString());
                    // Apply the best found PID values to AConstants for potential use
                    applyPIDToAConstants(bestIndividualEver);
                } else {
                    telemetryA.addLine("No best parameters found.");
                }
                break;
        }

        // Display general telemetry
        telemetryA.addData("GA State", currentState);
        telemetryA.addData("Generation", currentGeneration + "/" + MAX_GENERATIONS);
        if (bestIndividualEver != null) {
            telemetryA.addData("Overall Best Fitness", bestIndividualEver.fitness);
        }
        follower.telemetryDebug(telemetryA); // Pedro Pathing's debug telemetry
        telemetryA.update();
    }

    /**
     * Initializes the first population with random PID parameters.
     */
    private void initializePopulation() {
        population = new ArrayList<>();
        for (int i = 0; i < POPULATION_SIZE; i++) {
            population.add(new Individual());
        }
        telemetryA.addData("Population Initialized", POPULATION_SIZE + " individuals");
    }

    /**
     * Calculates the fitness of an individual.
     * THIS IS THE MOST CRITICAL PART YOU NEED TO IMPLEMENT.
     *
     * In a real robot, this would involve:
     * 1. Running the robot with the given PID parameters on a test path.
     * 2. Collecting performance metrics during the run (e.g., translational error, heading error,
     * overshoot, time to complete, smoothness of motion).
     * 3. Combining these metrics into a single fitness score.
     * A lower fitness score should indicate better performance.
     *
     * @param individual The individual (PID parameter set) to evaluate.
     * @return The calculated fitness score.
     */
    private double calculateFitness(Individual individual) {
        // --- YOUR IMPLEMENTATION HERE ---
        // This is a conceptual placeholder.
        // You would need to access follower's performance data or your own sensor data.

        // Example conceptual metrics:
        // double avgTranslationalError = follower.getAverageTranslationalError(); // You'd need to track this
        // double maxOvershoot = follower.getMaxOvershoot(); // You'd need to track this
        // double pathCompletionTime = runtime.seconds(); // Time taken for the current path run
        // double avgHeadingError = follower.getAverageHeadingError(); // You'd need to track this

        // A simple example fitness function (lower is better):
        // This is highly simplified and for demonstration only.
        // You would need to weight these based on what's important for your robot.
        // For example, if minimizing overshoot is critical, give it a higher weight.
        double conceptualError = follower.getTranslationalError().getMagnitude() + Math.abs(follower.getHeadingError()) + Math.abs(follower.getDriveVelocityError());
        double conceptualOvershoot = 0; // Placeholder, you'd calculate this from actual path data
        double conceptualTimePenalty = runtime.seconds() * 0.1; // Penalize longer times

        // This is a dummy fitness function for compilation and demonstration.
        // Replace with actual performance metrics from your robot.
        double fitness = conceptualError * 100 + conceptualOvershoot * 50 + conceptualTimePenalty;

        // Add a small random component to simulate real-world variability and prevent
        // all individuals from having identical fitness if performance is perfect.
        fitness += random.nextDouble() * 0.01;

        telemetryA.addData("Calculated Fitness", fitness);
        return fitness;
    }

    /**
     * Updates the overall best individual found so far.
     * @param currentIndividual The individual just evaluated.
     */
    private void updateBestIndividual(Individual currentIndividual) {
        if (bestIndividualEver == null || currentIndividual.fitness < bestIndividualEver.fitness) {
            bestIndividualEver = currentIndividual.clone();
            telemetryA.addLine("NEW BEST FOUND: " + bestIndividualEver.toString());
        }
    }

    /**
     * Gets the best fitness score within the current population.
     * @return The best fitness score.
     */
    private double getBestFitnessInCurrentGeneration() {
        double bestFitness = Double.POSITIVE_INFINITY;
        for (Individual individual : population) {
            if (individual.fitness < bestFitness) {
                bestFitness = individual.fitness;
            }
        }
        return bestFitness;
    }

    /**
     * Applies the PID parameters from an Individual to the static AConstants class.
     * This makes the values accessible to the Follower.
     * @param individual The individual whose PID parameters should be applied.
     */
    private void applyPIDToAConstants(Individual individual) {
        AConstants.P_TRANSLATIONAL = individual.pidParams[0];
        AConstants.I_TRANSLATIONAL = individual.pidParams[1];
        AConstants.D_TRANSLATIONAL = individual.pidParams[2];

        AConstants.P_HEADING = individual.pidParams[3];
        AConstants.I_HEADING = individual.pidParams[4];
        AConstants.D_HEADING = individual.pidParams[5];

        // Assuming AConstants has these for a generic "drive" PID.
        // If your system doesn't use these, you can remove them.
        // These might be used for individual motor PIDs or a master drive power scaling.
        AConstants.P_DRIVE = individual.pidParams[6];
        AConstants.I_DRIVE = individual.pidParams[7];
        AConstants.D_DRIVE = individual.pidParams[8];
    }

    /**
     * Evolves the population to the next generation using selection, crossover, and mutation.
     */
    private void evolvePopulation() {
        List<Individual> newPopulation = new ArrayList<>();

        // Elitism: Keep the best individual from the previous generation
        if (bestIndividualEver != null) {
            newPopulation.add(bestIndividualEver.clone());
        }

        // Fill the rest of the new population through genetic operations
        while (newPopulation.size() < POPULATION_SIZE) {
            // Selection: Choose two parents using Tournament Selection
            Individual parent1 = selectParent(population);
            Individual parent2 = selectParent(population);

            // Crossover: Create two children from the parents
            Individual[] children = crossover(parent1, parent2);

            // Mutation: Mutate the children
            Individual child1 = mutate(children[0]);
            Individual child2 = mutate(children[1]);

            // Add children to the new population
            newPopulation.add(child1);
            if (newPopulation.size() < POPULATION_SIZE) {
                newPopulation.add(child2);
            }
        }
        population = newPopulation;
    }

    /**
     * Selects a parent individual using Tournament Selection.
     * @param currentPopulation The current list of individuals to select from.
     * @return The selected parent individual.
     */
    private Individual selectParent(List<Individual> currentPopulation) {
        int tournamentSize = 3; // Number of individuals in each tournament
        Individual bestInTournament = null;

        for (int i = 0; i < tournamentSize; i++) {
            int randomIndex = random.nextInt(currentPopulation.size());
            Individual randomIndividual = currentPopulation.get(randomIndex);
            if (bestInTournament == null || randomIndividual.fitness < bestInTournament.fitness) {
                bestInTournament = randomIndividual;
            }
        }
        return bestInTournament;
    }

    /**
     * Performs single-point crossover between two parent individuals to create two children.
     * @param parent1 The first parent.
     * @param parent2 The second parent.
     * @return An array containing two new child individuals.
     */
    private Individual[] crossover(Individual parent1, Individual parent2) {
        double[] child1Params = new double[parent1.pidParams.length];
        double[] child2Params = new double[parent1.pidParams.length];

        // Choose a random crossover point (excluding the very first and last gene)
        int crossoverPoint = random.nextInt(parent1.pidParams.length - 2) + 1; // Ensures point is not 0 or last index

        for (int i = 0; i < parent1.pidParams.length; i++) {
            if (i < crossoverPoint) {
                child1Params[i] = parent1.pidParams[i];
                child2Params[i] = parent2.pidParams[i];
            } else {
                child1Params[i] = parent2.pidParams[i];
                child2Params[i] = parent1.pidParams[i];
            }
        }
        return new Individual[]{new Individual(child1Params), new Individual(child2Params)};
    }

    /**
     * Mutates an individual's PID parameters with a given probability and strength.
     * @param individual The individual to mutate.
     * @return A new mutated individual.
     */
    private Individual mutate(Individual individual) {
        double[] mutatedParams = individual.pidParams.clone();
        for (int i = 0; i < mutatedParams.length; i++) {
            if (random.nextDouble() < MUTATION_RATE) {
                // Apply a random perturbation
                mutatedParams[i] += (random.nextDouble() * 2 - 1) * MUTATION_STRENGTH; // Value between -strength and +strength
                // Clamp the value within the defined PID range
                mutatedParams[i] = Math.max(PID_MIN_VALUE, Math.min(PID_MAX_VALUE, mutatedParams[i]));
            }
        }
        return new Individual(mutatedParams);
    }
}
