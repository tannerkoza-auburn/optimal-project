# optimal-project

optimal-project is a repository for the Optimal Estimation project.

## Installation

Clone the repo.

```sh
git clone git@github.com:tannerkoza/optimal-project.git
```

## Usage

Once cloned, open the project as the current folder in MATLAB and run the following in the Command Window:

```sh
startup
```
This adds all subfolders in the project to the MATLAB path. This same functionality is also automatically achieved when MATLAB starts in this project folder.

If other receiver data is needed, add the unparsed IMU `.bag` or `.txt` files to `data/rawData`. Then, run `dataParser` in the Command Window from the root folder. The parsed files will be saved in `data/structData` to be used in other scripts. 

## Contributing
Contributions are PROHIBITED as this is a graded assignment.

## License
This repo serves as a future reference to concepts learned in this class and as personal repo management practice. It is not meant to be cloned and turned in.

[MIT](https://choosealicense.com/licenses/mit/)
