# GitHub Actions CI for ROS 2 Learning

This directory contains GitHub Actions workflows for continuous integration of the ROS 2 learning project.

## Workflows

### ROS 2 Build (`ros2-build.yml`)

This workflow builds and tests the ROS 2 packages in this repository.

#### Trigger Events
- Push to `main` branch
- Pull requests to `main` branch
- Manual trigger via GitHub Actions UI

#### Jobs

1. **Build**: Builds and tests all ROS 2 packages in the repository
   - Uses the official ROS 2 Jazzy container
   - Installs necessary dependencies
   - Builds service packages
   - Builds action packages
   - Runs tests for all packages

## Adding New Packages

When adding new ROS 2 packages to the repository:

1. Add the package to the appropriate build step in `.github/workflows/ros2-build.yml`
2. Make sure to include any new dependencies in the "Install dependencies" step

## Troubleshooting

If the CI build fails:

1. Check the GitHub Actions logs for specific error messages
2. Ensure all dependencies are properly installed
3. Verify that your package builds locally before pushing 