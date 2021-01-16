import os
from numpy import array
import shutil


def create_output_dir(name):
    """create output directory for name airplane."""
    cwd = os.getcwd()
    path = cwd + '/src/airplanes/' + name + '/output'
    if os.path.exists(path):
        shutil.rmtree(path)
    os.mkdir(path)
    return


def load_aero_model(name):
    """get aero model .txt file from output directory."""
    cwd = os.getcwd()
    path = cwd + '/src/airplanes/' + name
    file = open(path + "/model.txt", "r")
    contents = file.read()
    out = eval(contents)
    file.close()
    return out


def model_exists(name):
    """save aero model .txt file to output directory."""
    cwd = os.getcwd()
    path = cwd + '/src/airplanes/' + name + '/model.txt'
    return os.path.exists(path)


def plot_or_save(plt, plot, save, plane_name, name):
    cwd = os.getcwd()
    path = cwd + '/src/airplanes/' + plane_name + '/output'
    if plot:
        plt.show()
    if save:
        plt.savefig(path + ('/%s.png' % name))
        plt.close()
    return


def save_aero_model(model, name):
    """save aero model .txt file to output directory."""
    cwd = os.getcwd()
    path = cwd + '/src/airplanes/' + name
    f = open(path + "/model.txt", "w")
    f.write(str(model))
    f.close()
    return
