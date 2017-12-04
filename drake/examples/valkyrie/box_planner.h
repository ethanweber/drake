

namespace drake {
namespace examples {
namespace valkyrie {

using systems::controllers::plan_eval::PlanEvalBaseSystem;

class BoxPlanner : public PlanEvalBaseSystem {
  public:
    BoxPlanner(const RigidBodyTree<double>* robot,
               const std::string& alias_groups_file_name,
               const std::string& param_file_name, double dt);

  private:
    int get_num_extended_abstract_states() const override { return 1; }

    void DoExtendedCalcUnrestrictedUpdate(
        const systems::Context<double>& context,
        systems::State<double>* state) const override;

};

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake
