import os

from xacro_scene import XacroScene
from parse_graph import ParseGraph
from db_loader import DbLoader

from global_settings import SCENE_PG_FILENAME, SCENE_SEGMENTS_DIR


class SceneBuilder(object):

    def __init__(self):
        pass


    def generate(self, scene_dir, output_dir):
        scene_name = self.gen_scene_name_(scene_dir)

        xscene = XacroScene(scene_name, output_dir)
        pg = ParseGraph(self.get_pg_file_(scene_dir))

        self.add_to_scene_(xscene, pg, pg.get_root_idx())

        saved_dir = xscene.save()

        # if scene xacro files are succesfully saved
        if saved_dir != "":
            self.dump_bgm_files_(scene_dir, xscene.get_bgm_files())
            print("[INFO] Done! `{}` is saved at `{}`".format(scene_name, saved_dir))
        else:
            print("[INFO] Fail to convert `{}`".format(scene_dir))

    
    def add_to_scene_(self, xscene, pg, idx):
        # add current node into xacro scene
        target = pg.get_node(idx)
        parent = pg.get_parent(idx)

        xscene.add(target, parent)
        
        for c in pg.get_children_idx(idx):
            self.add_to_scene_(xscene, pg, c)


    def gen_scene_name_(self, scene_dir):
        """
        Generate the scene name
        """
        return scene_dir.split("/")[-1]


    def dump_bgm_files_(self, scene_dir, bgm_files):
        if len(bgm_files) == 0:
            return
        
        scene_bgm_dir = "{}/{}".format(scene_dir, SCENE_SEGMENTS_DIR)

        # check if background dump directory exists, if not, create it
        dump_bgm_dir = "/".join(bgm_files[0].split('/')[:-1])
        if not os.path.isdir(dump_bgm_dir):
            os.system("mkdir {}".format(dump_bgm_dir))
        
        for file in bgm_files:
            filename = file.split('/')[-1]
            os.system("cp {}/{} {}".format(scene_bgm_dir, filename, file))
        
        print("[INFO] Dumped background mesh files at {}".format(dump_bgm_dir))


    def get_pg_file_(self, scene_dir):
        return "{}/{}".format(scene_dir, SCENE_PG_FILENAME)


def build_scene(scene_dir, output_dir):
    sb = SceneBuilder()

    sb.generate(scene_dir, output_dir)


if __name__ == "__main__":
    scene_dir = "./input/scenenn_001"
    output_dir = "./output"

    build_scene(scene_dir, output_dir)
