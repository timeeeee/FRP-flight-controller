import Data.Geo.GPX

interaction :: FilePath -> FilePath -> IO ()
interaction = flip interactsGpx [
                -- set the copyright in the metadata
                setCopyright' (copyrightType "Fred" (Just "2009") (Just "BSD3")), 
                -- add a waypoint (home)
                usingWpts (home:),
                -- set the creator
                setCreator "Me!"]

home :: WptType
home = setDesc' "My house" .
       setCmt' "I live here" .
       setEle' 326.7 $
       wptType' (latitudeType (-27.69301))
                (longitudeType 152.718)