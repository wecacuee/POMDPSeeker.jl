module TestPOMDPSeeker

import POMDPSeeker:
    SourceSeeker,
    predict_nextmap,
    runSimulation,
    OccupancyGrid,
    convert_to_occgrid,
    convert_occgrid
using Test
using POMDPSimulators
using POMDPPolicies
using Serialization

@test SourceSeeker() != Nothing

@test size(predict_nextmap(rand(Float64, 256, 256), rand(Bool, 256, 256))) == (256, 256, 3)

function testOccgridConversion(filename="pyobject.jlobj")
    occgrd = deserialize(open(filename, read=true))
    occgrid_new = convert_to_occgrid(convert_occgrid(occgrd)...)
    return occgrd.data == occgrid_new.data
end
@test testOccgridConversion()
#@test runSimulation()

end # module
